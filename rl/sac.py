"""SAC (Soft Actor-Critic) 직접 구현 — PyTorch(CPU) 학습 + numpy 추론.

- 액터: MLP(obs → hidden → hidden → [μ, log σ]), tanh 로 [−1, 1] 에 가둔 가우시안.
- 크리틱: 쌍둥이 Q 를 한 텐서로 묶은 앙상블(baddbmm) — 작은 망에서 파이썬/연산 호출 수를 반으로 줄임.
- 자동 엔트로피(α), 소프트 타깃 갱신, 리플레이 버퍼.
- 선택: 행동 매끄러움 정칙화 CAPS (`caps_t`, `caps_s`, 기본 0 = 끔 → 표준 SAC 와 같음).
- `init_from(d, actor_path)`: 다른 실행(가짜 보드 사전학습)의 액터·크리틱·α 로 시작 (버퍼·옵티마이저는 새로).
- 체크포인트: 망·타깃·옵티마이저·α·카운터 (sac.pt) + 버퍼 (buffer.npz). 임시 파일에 쓴 뒤 바꿔치기.
- `NumpyPolicy`: 액터 가중치의 numpy 사본. 제어 루프(읽기 스레드)에서 torch 없이 추론 (약 10–20 µs).
- 정책 스냅샷: `save_policy(d)` → d/actor.pt (액터만), `load_policy(path)` 는 actor.pt 와 sac.pt 둘 다 읽음.
"""
from __future__ import annotations

import math
import os

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from .config import SacCfg

torch.set_num_threads(1)   # 작은 망은 스레드 1개가 가장 빠름 (측정: 다중 스레드는 오히려 느림)

LOG2 = math.log(2.0)


class ReplayBuffer:
    def __init__(self, obs_dim: int, act_dim: int, size: int):
        self.size = size
        self.o = np.zeros((size, obs_dim), np.float32)
        self.a = np.zeros((size, act_dim), np.float32)
        self.r = np.zeros((size, 1), np.float32)
        self.o2 = np.zeros((size, obs_dim), np.float32)
        self.d = np.zeros((size, 1), np.float32)
        self.ptr = 0
        self.n = 0

    def add(self, o, a, r, o2, d):
        i = self.ptr
        self.o[i] = o
        self.a[i] = a
        self.r[i] = r
        self.o2[i] = o2
        self.d[i] = d
        self.ptr = (i + 1) % self.size
        self.n = min(self.n + 1, self.size)

    def sample(self, batch: int, rng: np.random.Generator):
        idx = rng.integers(0, self.n, batch)
        return (torch.from_numpy(self.o[idx]), torch.from_numpy(self.a[idx]), torch.from_numpy(self.r[idx]),
                torch.from_numpy(self.o2[idx]), torch.from_numpy(self.d[idx]))

    def save(self, path: str):
        n = self.n
        tmp = path + '.tmp.npz'
        np.savez(tmp, o=self.o[:n], a=self.a[:n], r=self.r[:n], o2=self.o2[:n], d=self.d[:n],
                 ptr=self.ptr, n=n, size=self.size)
        os.replace(tmp, path)

    def load(self, path: str):
        z = np.load(path)
        n = int(z['n'])
        if n > self.size:
            raise ValueError('버퍼 크기가 체크포인트보다 작습니다')
        self.o[:n], self.a[:n], self.r[:n], self.o2[:n], self.d[:n] = z['o'], z['a'], z['r'], z['o2'], z['d']
        self.n = n
        self.ptr = int(z['ptr']) if int(z['size']) == self.size else n % self.size


class EnsembleMLP(nn.Module):
    """n 개의 독립 MLP 를 묶어 한 번에 계산: 입력 (n, B, i) → (n, B, o)."""

    def __init__(self, n: int, i: int, h: int, o: int):
        super().__init__()
        self.ws = nn.ParameterList()
        self.bs = nn.ParameterList()
        for a, b in ((i, h), (h, h), (h, o)):
            w = torch.empty(n, a, b)
            for k in range(n):
                lin = nn.Linear(a, b)
                w[k] = lin.weight.detach().T
            self.ws.append(nn.Parameter(w))
            self.bs.append(nn.Parameter(torch.zeros(n, 1, b)))

    def forward(self, x):
        h = F.relu(torch.baddbmm(self.bs[0], x, self.ws[0]))
        h = F.relu(torch.baddbmm(self.bs[1], h, self.ws[1]))
        return torch.baddbmm(self.bs[2], h, self.ws[2])


class Actor(nn.Module):
    def __init__(self, obs_dim, act_dim, h, log_std_min, log_std_max):
        super().__init__()
        self.l1 = nn.Linear(obs_dim, h)
        self.l2 = nn.Linear(h, h)
        self.l3 = nn.Linear(h, 2 * act_dim)
        self.act_dim = act_dim
        self.lmin, self.lmax = log_std_min, log_std_max

    def forward(self, o):
        h = F.relu(self.l1(o))
        h = F.relu(self.l2(h))
        out = self.l3(h)
        mu, ls = out[:, :self.act_dim], out[:, self.act_dim:]
        ls = self.lmin + 0.5 * (self.lmax - self.lmin) * (torch.tanh(ls) + 1)
        return mu, ls

    def sample(self, o):
        mu, ls = self(o)
        std = ls.exp()
        u = mu + std * torch.randn_like(mu)
        a = torch.tanh(u)
        logp = (-0.5 * ((u - mu) / std) ** 2 - ls - 0.5 * math.log(2 * math.pi)).sum(1, keepdim=True)
        logp = logp - (2 * (LOG2 - u - F.softplus(-2 * u))).sum(1, keepdim=True)
        return a, logp


class NumpyPolicy:
    """액터의 numpy 사본 (torch 를 거치지 않는 추론)."""

    def __init__(self, actor: Actor, seed: int = 0):
        self.rng = np.random.default_rng(seed)
        self.load(actor)

    def load(self, actor: Actor):
        with torch.no_grad():
            self.W = [l.weight.detach().numpy().T.astype(np.float64).copy() for l in (actor.l1, actor.l2, actor.l3)]
            self.b = [l.bias.detach().numpy().astype(np.float64).copy() for l in (actor.l1, actor.l2, actor.l3)]
        self.act_dim = actor.act_dim
        self.lmin, self.lmax = actor.lmin, actor.lmax

    def mu_logstd(self, o):
        W, b = self.W, self.b
        h = np.maximum(o @ W[0] + b[0], 0.0)
        h = np.maximum(h @ W[1] + b[1], 0.0)
        out = h @ W[2] + b[2]
        mu, ls = out[:self.act_dim], out[self.act_dim:]
        ls = self.lmin + 0.5 * (self.lmax - self.lmin) * (np.tanh(ls) + 1)
        return mu, ls

    def act(self, o, deterministic: bool = False):
        mu, ls = self.mu_logstd(np.asarray(o, np.float64))
        if deterministic:
            return np.tanh(mu)
        return np.tanh(mu + np.exp(ls) * self.rng.standard_normal(mu.shape))


class SAC:
    def __init__(self, obs_dim: int, act_dim: int, cfg: SacCfg):
        self.cfg = cfg
        self.obs_dim, self.act_dim = obs_dim, act_dim
        torch.manual_seed(cfg.seed)
        self.rng = np.random.default_rng(cfg.seed)
        h = cfg.hidden
        self.actor = Actor(obs_dim, act_dim, h, cfg.log_std_min, cfg.log_std_max)
        self.critic = EnsembleMLP(2, obs_dim + act_dim, h, 1)
        self.target = EnsembleMLP(2, obs_dim + act_dim, h, 1)
        self.target.load_state_dict(self.critic.state_dict())
        for p in self.target.parameters():
            p.requires_grad_(False)
        self._cparams = list(self.critic.parameters())
        self._tparams = list(self.target.parameters())
        self.log_alpha = torch.tensor(math.log(cfg.init_alpha), requires_grad=True)
        self.opt_actor = torch.optim.Adam(self.actor.parameters(), lr=cfg.lr, fused=True)
        self.opt_critic = torch.optim.Adam(self.critic.parameters(), lr=cfg.lr, fused=True)
        self.opt_alpha = torch.optim.Adam([self.log_alpha], lr=cfg.lr)
        self.buffer = ReplayBuffer(obs_dim, act_dim, cfg.buffer_size)
        self.updates = 0
        self.last = dict(q_loss=float('nan'), pi_loss=float('nan'), alpha=cfg.init_alpha, entropy=float('nan'),
                         q_mean=float('nan'))
        self.policy = NumpyPolicy(self.actor, cfg.seed + 1)

    @property
    def alpha(self) -> float:
        return float(self.log_alpha.detach().exp())

    def _q(self, net, o, a):
        x = torch.cat([o, a], 1)
        return net(x.unsqueeze(0).expand(2, -1, -1))     # (2, B, 1)

    def update(self, n: int = 1):
        """경사 갱신 n 번."""
        cfg = self.cfg
        if self.buffer.n < cfg.batch:
            return 0
        tp, cp = self._tparams, self._cparams
        for i in range(n):
            o, a, r, o2, d = self.buffer.sample(cfg.batch, self.rng)
            alpha = self.log_alpha.exp().detach()
            with torch.no_grad():
                a2, logp2 = self.actor.sample(o2)
                qt = self._q(self.target, o2, a2).min(0)[0]
                y = r + cfg.gamma * (1 - d) * (qt - alpha * logp2)
            q = self._q(self.critic, o, a)
            q_loss = ((q - y) ** 2).mean() * 2
            self.opt_critic.zero_grad(set_to_none=True)
            q_loss.backward()
            self.opt_critic.step()

            an, logp = self.actor.sample(o)
            for p in cp:
                p.requires_grad_(False)          # 액터 손실의 역전파가 크리틱 가중치 기울기를 만들지 않게
            qn = self._q(self.critic, o, an).min(0)[0]
            for p in cp:
                p.requires_grad_(True)
            pi_loss = (alpha * logp - qn).mean()
            if cfg.caps_t > 0 or cfg.caps_s > 0:
                pi_loss = pi_loss + self._caps(o, o2, d)
            self.opt_actor.zero_grad(set_to_none=True)
            pi_loss.backward()
            self.opt_actor.step()

            # α 손실 −log α·(log π + H*) 의 기울기는 −(E[log π] + H*): 역전파 없이 직접 넣음
            lp = logp.detach().mean()
            self.log_alpha.grad = -(lp + cfg.target_entropy)
            self.opt_alpha.step()

            with torch.no_grad():
                torch._foreach_lerp_(tp, cp, cfg.tau)
            self.updates += 1
        self.last = dict(q_loss=float(q_loss.detach()), pi_loss=float(pi_loss.detach()), alpha=float(alpha),
                         entropy=float(-lp), q_mean=float(q.detach().mean()))
        if cfg.caps_t > 0 or cfg.caps_s > 0:
            self.last['caps'] = self._caps_last
        return n

    def _caps(self, o, o2, d):
        """행동 매끄러움 정칙화 (CAPS, Mysore et al. 2021) — 결정적 행동 tanh(μ) 에 대해
        시간 항 λ_T·‖π(o) − π(o')‖² (다음 결정 관측, 종단 전이 제외) + 공간 항 λ_S·‖π(o) − π(o + ε)‖² (ε ~ N(0, σ²), 정규화 관측).
        보상 벌점(w_du·w_ddu)과 달리 Q 추정 잡음을 거치지 않고 액터를 직접 매끄럽게 함. 기본 끔 (λ = 0)."""
        cfg = self.cfg
        m0 = torch.tanh(self.actor(o)[0])
        loss = torch.zeros(())
        if cfg.caps_t > 0:
            m1 = torch.tanh(self.actor(o2)[0])
            loss = loss + cfg.caps_t * (((m0 - m1) ** 2).sum(1, keepdim=True) * (1 - d)).mean()
        if cfg.caps_s > 0:
            ms = torch.tanh(self.actor(o + cfg.caps_eps * torch.randn_like(o))[0])
            loss = loss + cfg.caps_s * ((m0 - ms) ** 2).sum(1).mean()
        self._caps_last = float(loss.detach())
        return loss

    def update_critic(self, n: int = 1):
        """크리틱만 경사 갱신 n 번 (액터·α 고정). 분기(rl.fork) 때 크리틱을 지금 액터에 다시 맞추는 용도.
        갱신 카운터(updates)는 올리지 않습니다. 마지막 q 손실을 돌려줌."""
        cfg = self.cfg
        if self.buffer.n < cfg.batch:
            return float('nan')
        tp, cp = self._tparams, self._cparams
        alpha = self.log_alpha.exp().detach()
        q_loss = torch.tensor(float('nan'))
        for _ in range(n):
            o, a, r, o2, d = self.buffer.sample(cfg.batch, self.rng)
            with torch.no_grad():
                a2, logp2 = self.actor.sample(o2)
                qt = self._q(self.target, o2, a2).min(0)[0]
                y = r + cfg.gamma * (1 - d) * (qt - alpha * logp2)
            q = self._q(self.critic, o, a)
            q_loss = ((q - y) ** 2).mean() * 2
            self.opt_critic.zero_grad(set_to_none=True)
            q_loss.backward()
            self.opt_critic.step()
            with torch.no_grad():
                torch._foreach_lerp_(tp, cp, cfg.tau)
        return float(q_loss.detach())

    def sync_policy(self):
        """갱신 뒤 numpy 사본을 새로 고침 (에피소드 시작 전에만 호출)."""
        self.policy.load(self.actor)

    # ---- 체크포인트
    def save(self, d: str, extra: dict | None = None):
        os.makedirs(d, exist_ok=True)
        state = dict(actor=self.actor.state_dict(), critic=self.critic.state_dict(), target=self.target.state_dict(),
                     opt_actor=self.opt_actor.state_dict(), opt_critic=self.opt_critic.state_dict(),
                     opt_alpha=self.opt_alpha.state_dict(), log_alpha=self.log_alpha.detach().clone(),
                     updates=self.updates, rng=self.rng.bit_generator.state, torch_rng=torch.get_rng_state(),
                     extra=extra or {})
        tmp = os.path.join(d, 'sac.pt.tmp')
        torch.save(state, tmp)
        os.replace(tmp, os.path.join(d, 'sac.pt'))
        self.buffer.save(os.path.join(d, 'buffer.npz'))

    def load(self, d: str) -> dict:
        state = torch.load(os.path.join(d, 'sac.pt'), weights_only=False)
        self.actor.load_state_dict(state['actor'])
        self.critic.load_state_dict(state['critic'])
        self.target.load_state_dict(state['target'])
        self.opt_actor.load_state_dict(state['opt_actor'])
        self.opt_critic.load_state_dict(state['opt_critic'])
        self.opt_alpha.load_state_dict(state['opt_alpha'])
        with torch.no_grad():
            self.log_alpha.copy_(state['log_alpha'])
        self.updates = state['updates']
        self.rng.bit_generator.state = state['rng']
        torch.set_rng_state(state['torch_rng'])
        bp = os.path.join(d, 'buffer.npz')
        if os.path.exists(bp):
            self.buffer.load(bp)
        self.sync_policy()
        return state.get('extra', {})

    def init_from(self, ckpt_dir: str, actor_path: str | None = None) -> dict:
        """사전학습 이어받기: ckpt_dir/sac.pt 의 크리틱·타깃·α (+ 액터; actor_path 를 주면 그 스냅샷의 액터).
        버퍼·옵티마이저 상태·갱신 카운터는 가져오지 않습니다 (다른 플랜트의 전이를 섞지 않음)."""
        state = torch.load(os.path.join(ckpt_dir, 'sac.pt'), weights_only=False, map_location='cpu')
        self.critic.load_state_dict(state['critic'])
        self.target.load_state_dict(state['target'])
        if actor_path:
            sd, info = load_actor_state(actor_path)
        else:
            sd, info = state['actor'], dict(file=os.path.join(ckpt_dir, 'sac.pt'), updates=state.get('updates'))
        self.actor.load_state_dict(sd)
        alpha = float(info.get('alpha') or float(state['log_alpha'].exp()))
        with torch.no_grad():
            self.log_alpha.copy_(torch.tensor(math.log(alpha)))
        self.sync_policy()
        return dict(critic=os.path.join(ckpt_dir, 'sac.pt'), actor=info['file'], alpha=alpha,
                    src_updates=info.get('updates'))

    # ---- 정책만 (스냅샷·재생용: 액터 가중치만, 버퍼·크리틱·옵티마이저 없음)
    def save_policy(self, d: str):
        """d/actor.pt 에 액터 가중치와 망 모양을 저장 (수십 KB)."""
        os.makedirs(d, exist_ok=True)
        state = dict(format='aero_rl_actor_1', actor=self.actor.state_dict(), obs_dim=self.obs_dim,
                     act_dim=self.act_dim, hidden=self.cfg.hidden, log_std_min=self.cfg.log_std_min,
                     log_std_max=self.cfg.log_std_max, alpha=self.alpha, updates=self.updates)
        tmp = os.path.join(d, 'actor.pt.tmp')
        torch.save(state, tmp)
        os.replace(tmp, os.path.join(d, POLICY_FILE))

    def load_policy(self, path: str) -> dict:
        """actor.pt(스냅샷) 또는 sac.pt(체크포인트)에서 액터만 읽어 numpy 사본까지 갱신. 정보 dict 를 돌려줌."""
        sd, info = load_actor_state(path)
        self.actor.load_state_dict(sd)
        self.sync_policy()
        return info


POLICY_FILE = 'actor.pt'


def load_actor_state(path: str):
    """경로(파일 또는 폴더)에서 액터 state_dict 와 {file, updates, ...} 를 읽습니다.
    폴더면 actor.pt 를 먼저, 없으면 sac.pt 를 찾습니다."""
    if os.path.isdir(path):
        for name in (POLICY_FILE, 'sac.pt'):
            if os.path.exists(os.path.join(path, name)):
                path = os.path.join(path, name)
                break
        else:
            raise FileNotFoundError(f'{path} 에 {POLICY_FILE} 또는 sac.pt 가 없습니다')
    state = torch.load(path, weights_only=False, map_location='cpu')
    if 'actor' not in state:
        raise ValueError(f'{path}: 액터 가중치가 없습니다')
    info = dict(file=path, updates=state.get('updates'))
    if 'alpha' in state:
        info['alpha'] = state['alpha']
    elif 'log_alpha' in state:
        info['alpha'] = float(state['log_alpha'].exp())
    return state['actor'], info
