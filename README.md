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

Most consumers only need the compiled module: see [Prebuilt wheels](release/README.md#prebuilt-wheels).

## What this is

The drawing-placement engine of the doodlebot live server, in Rust with a Python
extension: given a canvas, the drawings on it and the robots drawing right now,
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
  Cargo.toml / Cargo.lock crate `canvas_placement` (rlib + cdylib), `placement` CLI (feature `cli`), Python ext (feature `python`)
  src/                    the engine (module map in release/README.md)
  pyproject.toml          maturin config for the wheel (version comes from Cargo.toml)
  README.md               consumer documentation
.github/workflows/        CI: build-release.yml (below) + suede's publish workflow
.suede/core/              suede's maintainer tools (a subrepo; update with `bash .suede/core/sync.sh`)
```

## Development

Needs a Rust toolchain, and `maturin` for the Python extension.

```
cargo test --release --manifest-path release/Cargo.toml
cargo build --release --features cli --manifest-path release/Cargo.toml
maturin develop --release -m release/Cargo.toml     # into the active virtualenv
```

Push with `bash .suede/core/push.sh`: when `release/` changes on `main`, suede
publishes it to the `release` branch.

## Releases

[`.github/workflows/build-release.yml`](.github/workflows/build-release.yml) runs
the Rust tests, builds abi3 wheels (Linux x86_64/aarch64, macOS universal2,
Windows x64) and an sdist from `release/`, smoke-tests each wheel, and attaches
them to a GitHub release.

1. Bump `version` in `release/Cargo.toml` (the wheel takes its version from there).
2. Push a tag `v<version>` (the workflow refuses a tag that doesn't match), or run the
   workflow by hand from the Actions tab, which tags the current commit as `v<version>`.

Pull requests touching `release/` run the same tests and builds without publishing.
Built artifacts never go into `release/`; it holds source only.

Not yet built: a wasm package (for use in the browser). `src/clock.rs` already
compiles without `std::time` on wasm32; the bindings and a build job are what's left.
