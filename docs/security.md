# Security Considerations

## Threat model

The pipeline runs locally, generates solver input files, and optionally invokes external solvers. The security boundaries are therefore: (a) input files -> Python models, (b) Python models -> generated solver decks, (c) Python -> subprocess execution.

## Controls

1. **Input validation.** All physical inputs pass through frozen dataclasses whose `__post_init__` checks enforce positivity and geometric consistency (e.g., wall thickness < outer radius, thrust-to-weight >= 1). Physically impossible missions raise `SizingError` before any computation.
2. **Subprocess hygiene.** External executables (`ccx`, `simpleFoam`, `blockMesh`) are resolved with `shutil.which` and invoked with argument lists, never interpolated shell strings; there is no shell injection surface. Solver executables are optional: when absent the pipeline uses the in-repo validated models instead of failing.
3. **Generated artifacts.** `.inp` decks and OpenFOAM dictionaries embed only validated numeric parameters; no user text is interpolated into solver input files.
4. **Docker.** The compose service has no privileged mode, no host network, and mounts only `./results` read-write for outputs.
5. **No network.** No stage performs network I/O; the pipeline is safe to run offline.

## Known limitations

- Requirement JSON files loaded by users are trusted; there is no schema-validation layer beyond the dataclass checks for externally supplied files.
- Generated OpenFOAM cases assume a trusted local environment (OpenFOAM dictionaries are not sandboxed by the pipeline).
- The Docker image installs pinned wheels from PyPI and inherits their supply-chain trust.

## Vulnerability disclosure

None known. Report issues via the repository tracker.
