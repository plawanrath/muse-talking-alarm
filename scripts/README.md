# scripts

Host-side helper scripts go here.

Tomorrow: copy the macOS ESP-IDF v6.0.1 setup script here and commit it,
so the repo is self-contained:

    cp <path-to>/esp-idf-v6.0.1-macos-setup.sh scripts/
    git add scripts/esp-idf-v6.0.1-macos-setup.sh
    git commit -m "Add macOS ESP-IDF v6.0.1 setup script"

Secrets (Wi-Fi credentials, tokens) never go in scripts — see the root
`.gitignore`.
