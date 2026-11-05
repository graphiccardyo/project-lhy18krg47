$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Add-Type -Path (Join-Path $projectRoot 'MpuAttitudeView.cs') -ReferencedAssemblies @('System.dll', 'System.Windows.Forms.dll', 'System.Drawing.dll')
$half = [math]::Sqrt(0.5)
$cases = @(
    @{ Name = 'identity'; Q = @(1,0,0,0); V = @(1,2,3); Expected = @(1,2,3) },
    @{ Name = 'positive X 90'; Q = @($half,$half,0,0); V = @(0,1,0); Expected = @(0,0,1) },
    @{ Name = 'positive Y 90'; Q = @($half,0,$half,0); V = @(0,0,1); Expected = @(1,0,0) },
    @{ Name = 'positive Z 90'; Q = @($half,0,0,$half); V = @(1,0,0); Expected = @(0,1,0) },
    @{ Name = 'X 180 inverted'; Q = @(0,1,0,0); V = @(0,0,1); Expected = @(0,0,-1) }
)
foreach ($case in $cases) {
    $q = $case.Q; $v = $case.V
    $actual = [MpuAttitudeView]::RotateVector($q[0],$q[1],$q[2],$q[3],$v[0],$v[1],$v[2])
    for ($i = 0; $i -lt 3; $i++) { if ([math]::Abs($actual[$i] - $case.Expected[$i]) -gt 0.000001) { throw ('Rotation failed: ' + $case.Name) } }
    'PASS ' + $case.Name
}
$view = New-Object MpuAttitudeView
try {
    $view.SetOrientation($half,0,0,$half)
    $view.SetYawReference(90)
    $actual = $view.GetDisplayVector(1,0,0)
    if ([math]::Abs($actual[0]-1) -gt 0.000001 -or [math]::Abs($actual[1]) -gt 0.000001) { throw 'Yaw reference reset sign mismatch' }
    'PASS yaw reference reset preserves world up'
} finally { $view.Dispose() }
