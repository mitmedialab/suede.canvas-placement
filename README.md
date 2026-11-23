# Suede.canvas Placement

This repo is a [suede dependency](https://github.com/pmalacho-mit/suede). 

To see the installable source code, please checkout the [release branch](https://github.com/mitmedialab/suede.canvas-placement/tree/release).

## Installation

```bash
bash <(curl -fsSL https://suede.sh/install/release) --repo mitmedialab/suede.canvas-placement
```

Run it where you want the dependency. It installs `./suede.canvas-placement`, stages it, and prints
what else (if anything) has to be installed beside it.

<details>
<summary>
See alternative to using <a href="https://github.com/pmalacho-mit/suede#suedesh">suede.sh</a> script proxy
</summary>

```bash
bash <(curl -fsSL https://raw.githubusercontent.com/pmalacho-mit/suede/refs/heads/main/scripts/install/release.sh) --repo mitmedialab/suede.canvas-placement
```

</details>

Most consumers only need the compiled module: see [prebuilt builds](release/README.md#prebuilt-builds)
(npm packages for the browser and Node) and [prebuilt wheels](release/README.md#prebuilt-wheels) (Python).

## What this is

The drawing-placement engine of the doodlebot live server, in Rust with Python
and WebAssembly builds: given a canvas, the drawings on it and the robots drawing right now,
where does a new drawing go? It is stateless — every call rebuilds occupancy
from the drawings it's given. [release/README.md](release/README.md) documents
the API.

It replaced a Python/numpy/PIL engine in the server. Per placement, on the
server's packed-canvas and multi-robot tests:

| | Python engine | This engine |
|---|---|---|
| mean per placement (full size, or shrink-to-fit) | 84.5 ms | 7.1 ms |

## Layout

```
release/                  shipped source (the `release` branch)
  Cargo.toml / Cargo.lock crate `canvas_placement` (rlib + cdylib): `placement` CLI (feature `cli`),
                          Python ext (feature `python`), WebAssembly bindings (feature `wasm`)
  src/                    the engine (module map in release/README.md)
  pyproject.toml          maturin config for the wheel (version comes from Cargo.toml)
  build_wasm.sh           builds the wasm module + JS glue into release/pkg-web/ and release/pkg-node/ (gitignored)
  README.md               consumer documentation
web/index.html            browser demo (deployed to GitHub Pages with the web build)
.devcontainer/            development container (below)
.github/workflows/        CI: build-release.yml (below) + suede's publish workflow
.suede/core/              suede's maintainer tools (a subrepo; update with `bash .suede/core/sync.sh`)
```

## Development

`.devcontainer/` sets up Ubuntu 24.04 with Rust (stable + `wasm32-unknown-unknown`),
Python 3.12, Node 22, `wasm-bindgen-cli` pinned to the version in `release/Cargo.lock`,
`maturin`, Binaryen 123 and git-subrepo (for suede's scripts). Open the repo in it, or
run `.devcontainer/post-create.sh` on a machine that has the same base tools.

```
cargo test --release --manifest-path release/Cargo.toml
cargo build --release --features cli --manifest-path release/Cargo.toml
maturin develop --release -m release/Cargo.toml     # into the active virtualenv
./release/build_wasm.sh                             # wasm + JS glue
python3 -m http.server 8000                         # then open http://localhost:8000/web/?wasm=local
```

The demo loads the wasm from GitHub Pages by default; `?wasm=local` uses your
`release/pkg-web/` build instead, and `?wasm=<base url>` any other copy.

Push with `bash .suede/core/push.sh`: when `release/` changes on `main`, suede
publishes it to the `release` branch.

## Releases

[`.github/workflows/build-release.yml`](.github/workflows/build-release.yml) runs
the Rust tests, then builds from `release/` and attaches to a GitHub release: the
Node and browser wasm packages (`npm install <url>`), abi3 wheels (Linux
x86_64/aarch64, macOS universal2, Windows x64) and an sdist. Each build is
smoke-tested before publishing.

Browsers can't load release assets (GitHub serves them without CORS headers), so
the workflow also deploys the browser build plus the `web/` demo to GitHub Pages:
https://mitmedialab.github.io/suede.canvas-placement/. This needs Pages turned on
once, with *Settings → Pages → Source: GitHub Actions*.

1. Bump `version` in `release/Cargo.toml` (the wheel takes its version from there).
2. Push a tag `v<version>` (the workflow refuses a tag that doesn't match), or run the
   workflow by hand from the Actions tab, which tags the current commit as `v<version>`.

Pull requests touching `release/` run the same tests and builds without publishing.
Built artifacts never go into `release/`; it holds source only.

## License

MIT (declared in `release/Cargo.toml` and `release/pyproject.toml`).
