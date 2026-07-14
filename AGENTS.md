# Repository-wide path rules

- All committed production code and configuration must be portable: never hard-code machine-specific absolute paths such as `/home/<user>/...` or `/data/...`.
- Repository assets must be written as relative paths. Resolve a path declared in a configuration file relative to that configuration file; resolve code-owned assets from the repository root derived from `__file__`.
- External workspaces and runtimes must be supplied through environment variables or CLI options. A portable relative workspace layout may be used as the default, but a developer-specific absolute fallback must not be committed.
- Keep these rules when adding scripts, YAML/JSON/TOML configuration, tests, and documentation commands. Temporary absolute paths created by a test framework and explicit user-provided runtime paths are allowed.
