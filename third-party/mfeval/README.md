# MFeval (vendored third-party code)

Magic Formula (MF 5.2 / 6.1 / 6.2) tire-model evaluation for MATLAB. This is
the tire-force engine behind `lts.components.Tire.PacejkaTire`; it is vendored
so a fresh clone plus `addpath` is a complete, reproducible setup (and so CI
runners do not need a manual File Exchange install — MathWorks blocks
scripted downloads).

- Upstream: <https://www.mathworks.com/matlabcentral/fileexchange/63618-mfeval>
- Project site: <https://mfeval.wordpress.com>
- Author: Marco Furlan
- Version vendored: **4.3.1** (12-Mar-2021, the latest upstream release;
  matches `Contents.m`)
- Contents: source files only (`mfeval.m`, `mfevalroot.m`, `Contents.m`, and
  the `+mfeval` package). The upstream Simulink library, HTML docs, and
  examples are not needed by lts and are not included.
- Upstream is unmodified. Local modifications are forbidden — update by
  re-vendoring a newer upstream release.

## License

The upstream source carries no embedded license header and its File Exchange
"View License" text is only visible interactively. The project describes
itself as "an open source MATLAB toolbox". If you redistribute this repository,
check the upstream File Exchange page's license link before further
redistribution of this folder.
