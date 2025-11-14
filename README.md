# Garces Memorial High School `push-back`

> GMHS's Team `28001A` 2025-2026 V5RC Push-Back entry

![robot](/photos/robot.jpeg)

# Compiling and Usage

## Getting Started (Windows)

Follow the instructions [here](https://www.rust-lang.org/tools/install) to
install `rustup`.

Run the following commands in Powershell to set up your PC for development on
Windows.

- Switch to the `nightly` rust toolchain and add the `rust-src` component:

  ```console
  rustup default nightly
  rustup component add rust-src
  ```

- Install cargo-v5:

  ```console
  cargo install cargo-v5
  ```

## Getting Started (macOS)

Follow the instructions [here](https://www.rust-lang.org/tools/install) to
install `rustup` on your Mac.

Run the following commands in a terminal window to setup development with
vexide.

- Open a terminal and configure `rustup` to build for the V5's platform target:

- Switch to the `nightly` rust toolchain and add the `rust-src` component:

  ```console
  rustup default nightly
  rustup component add rust-src
  ```

- Install cargo-v5:

  ```console
  cargo install cargo-v5
  ```

## Getting Started (NixOS)

The Nix flake includes a devshell with every tool you need for building and
uploading vexide projects.

There is a `.envrc` file for Nix + Direnv users.

## Getting Started (Debian/Ubuntu Linux)

Follow the instructions [here](https://www.rust-lang.org/tools/install) to
install `rustup`. You may also prefer to install it from your system package
manager or by other means. Instructions on that can be found
[here](https://rust-lang.github.io/rustup/installation/other.html).

Run the following terminal commands to set up development on Debian or Ubuntu.

- Switch to the `nightly` rust toolchain and add the `rust-src` component:

  ```console
  rustup default nightly
  rustup component add rust-src
  ```

- Install cargo-v5:

  ```console
  cargo install cargo-v5
  ```

## Getting Started (Fedora Linux)

Run the following terminal commands to set up your PC for development on Fedora.

- Install Rust:

  ```console
  sudo dnf install rustup
  rustup-init -y --default-toolchain nightly
  ```

- Close and reopen the terminal, and finish installing vexide:

  ```console
  rustup component add rust-src
  cargo install cargo-v5
  ```

## Development

### Compiling and uploading to a VEX V5 robot

Use the cargo-v5 terminal utility to build and upload this vexide project.

```console
cargo v5 build
```

Use a USB cable to connect to your robot brain or to your controller before
using the `upload` subcommand to build and upload the project. Make sure to
specify a program slot.

```console
cargo v5 upload
```

### Viewing program output

You can view panic messages and calls to `println!()` using the terminal. Use a
USB cable to connect to your robot brain or controller, then start the terminal:

```console
cargo v5 terminal
```
