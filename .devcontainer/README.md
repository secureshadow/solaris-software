# Dev Container configuration: pick the right `devcontainer.json` for your OS

This repository includes multiple `devcontainer.json` presets under `json/`.  

Replace the default file at:
/workspaces/solaris-software/.devcontainer/devcontainer.json

with the OS-specific version from:
/workspaces/solaris-software/json/

## Steps
1. If the `devcontainer.json` is not correct, an error will be displayed. Select Edit devcontainer.json locally.

2. With the container opened locally, replace the `devcontainer.json` with the variant appropriate for the target operating system.

3. Perform a Rebuild / Reopen in Container.

**Note**: Because the configuration differs by operating system, issues are common—primarily due to path syntax differences across OSes. If any problems arise, contact the administrator or reach out via email.
