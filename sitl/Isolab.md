# Isolab

## Overview

Isolab, short for "Isolated Lab," is the SITL harness for Luna. The name is a
nod to the VM-first architecture: tests run inside an isolated Linux lab where
the harness can create namespaces, links, and routing rules without depending
on host privileges or host networking state.

The harness provides a reproducible Linux integration-test environment for
running real Luna software inside a NixOS VM while simulating the surrounding
network and board ecosystem in software.

The current harness focuses on the Flight/Servo dual-telemetry path:

- real `flight-computer`
- real `servo`
- simulated TEL routing topology (`flight`, `ftel`, `gtel`, `servo`)
- simulated SAM board traffic
- simulated GUI behavior through Servo's public HTTP and WebSocket APIs

The goal is to validate system behavior end to end, not just unit-level packet
encoding.

## High-Level Structure

The harness lives in [`sitl/isolab`](./isolab) and is split into a
small reusable library plus a thin binary entrypoint.

Main modules:

- [`args.rs`](./isolab/src/args.rs): CLI parsing and scenario selection.
- [`lab.rs`](./isolab/src/lab.rs): generic namespace/link/bridge helpers.
- [`process.rs`](./isolab/src/process.rs): process spawning and Python
  module staging.
- [`client.rs`](./isolab/src/client.rs): Servo HTTP/WebSocket client
  helpers, mappings, and telemetry assertions.
- [`components/sam.rs`](./isolab/src/components/sam.rs): internal SAM
  emulator process used by scenarios.
- [`topology/servo_flight.rs`](./isolab/src/topology/servo_flight.rs):
  the current concrete network topology and routing rules.
- [`scenarios/mod.rs`](./isolab/src/scenarios/mod.rs): scenario setup and
  scenario-specific assertions.
- [`lib.rs`](./isolab/src/lib.rs): library entrypoint that wires CLI to
  scenarios.
- [`main.rs`](./isolab/src/main.rs): thin Tokio binary entrypoint.

The NixOS VM checks that invoke the harness live in
[`sitl/default.nix`](./default.nix).

## Execution Model

The VM test flow is:

1. Nix builds the required binaries.
2. Nix boots one Linux VM.
3. The VM test script runs the harness binary inside the guest.
4. The harness creates Linux namespaces inside the guest.
5. The harness launches the real processes inside those namespaces.
6. The harness simulates missing hardware and GUI behavior.
7. The harness asserts on observable system behavior.

This keeps the host unprivileged while still allowing namespace, routing, and
iptables operations inside the guest.

## Current Topology

The current `servo_flight` topology creates four namespaces:

- `flight`
- `servo`
- `ftel`
- `gtel`

Network model:

- `flight` and `ftel` share a rocket-side Ethernet segment.
- `servo` and `gtel` share a ground-side Ethernet segment.
- those segments are joined by an `umbilical` veth pair
- `ftel` and `gtel` are joined by a `radio0` veth pair with MTU `255`

Behavior modeled:

- direct umbilical connectivity while the link is up
- DSCP-based flight policy routing toward FTEL
- FTEL/GTEL NAT rules for radio forwarding
- umbilical disconnect by bringing down the veth pair

The topology-specific code is intentionally isolated in
[`topology/servo_flight.rs`](./isolab/src/topology/servo_flight.rs). The
generic `Lab` does not know about Flight, Servo, TEL, or any specific IP plan.

## Generic `Lab`

[`lab.rs`](./isolab/src/lab.rs) is the reusable network orchestration
layer.

It currently supports:

- creating namespaces
- creating bridges
- attaching namespace NICs to bridges
- creating bridge-to-bridge links
- creating namespace-to-namespace veth links
- creating control links between the root namespace and a child namespace
- assigning addresses
- renaming interfaces
- bringing interfaces up
- setting MTU
- enabling IPv4 forwarding
- generic command execution
- cleanup helpers for links and namespaces

## Process Model

[`process.rs`](./isolab/src/process.rs) contains the generic process
supervision layer.

Current responsibilities:

- spawn a process in a target namespace
- attach stdout/stderr to log files
- manage process lifetime through `ManagedChild`
- stage Python extension modules into a temporary `PYTHONPATH`

Today, the real services are:

- `servo serve --volatile --quiet`
- `flight-computer --disable-gps desktop`

## Components

The harness currently has one explicit emulated component:

- SAM, in [`components/sam.rs`](./isolab/src/components/sam.rs)

The SAM emulator is run as an internal subprocess of the harness binary inside
the `flight` namespace. This keeps the emulator deployment path consistent with
the rest of the test system and avoids host/namespace packet-injection
ambiguity.

This module structure is intended to scale to future emulators such as:

- BMS
- AHRS
- other board protocol speakers

The recommended pattern is:

- one component module per board family
- one internal subprocess argument per component
- component-specific traffic generation isolated from scenario logic

## Servo Client Layer

[`client.rs`](./isolab/src/client.rs) acts as a simulated GUI client.

It owns:

- posting mappings to Servo
- activating the selected mapping configuration
- connecting to `/data/forward`
- connecting to `/data/forward?source=...`
- decoding streamed `VehicleState` JSON
- shared telemetry assertions

This keeps scenarios focused on behavior, not on raw HTTP/WebSocket plumbing.

## Scenarios

[`scenarios/mod.rs`](./isolab/src/scenarios/mod.rs) is the scenario layer.

Current scenarios:

- `default-source-umbilical`
- `radio-survives-disconnect`
- `vespula-radio-forwarding`

Each scenario:

1. creates the topology
2. launches the real processes
3. waits for readiness
4. applies mappings
5. starts the SAM emulator
6. subscribes to the required telemetry streams
7. performs scenario-specific assertions

## Nix Integration

[`sitl/default.nix`](./default.nix) provides the VM checks.

Current package sources:

- `self.packages.<system>.servo.servo`
- `self.packages.<system>.flight2.flight-computer`
- `self.packages.<system>.common.sequences`
- `self.packages.<system>.sitl.isolab`

Current checks:

- `default-source-umbilical`
- `radio-survives-disconnect`
- `vespula-radio-forwarding`
- `radio-without-sam`

The VM wrapper captures harness stdout/stderr and rethrows failures with the
full harness output so the top-level Nix failure is easier to interpret.

## How To Run It

Single scenario:

```bash
nix build .#checks.x86_64-linux.isolab.vespula-radio-forwarding -L
```

All current Servo/Flight SITL checks:

```bash
nix build .#checks.x86_64-linux.isolab.default-source-umbilical -L
nix build .#checks.x86_64-linux.isolab.radio-survives-disconnect -L
nix build .#checks.x86_64-linux.isolab.vespula-radio-forwarding -L
nix build .#checks.x86_64-linux.isolab.radio-without-sam -L
```

More broadly:

```bash
nix flake check -L
```

To run a scenario through the flake app entrypoint:

```bash
nix run .#isolab.vespula-radio-forwarding
```

## How To Add A New Scenario

If the topology is unchanged:

1. Add a new variant in [`args.rs`](./isolab/src/args.rs).
2. Add a new branch in [`scenarios/mod.rs`](./isolab/src/scenarios/mod.rs).
3. Reuse the existing setup helpers and Servo client helpers.
4. Add a new VM check entry in [`sitl/default.nix`](./default.nix).

If the scenario needs new simulated hardware:

1. Add a new component module under
   [`components/`](./isolab/src/components).
2. Add an internal subprocess entrypoint if needed.
3. Start that component from the scenario setup path.
4. Add the minimum mappings or protocol setup needed for the test.

If the scenario needs a different network shape:

1. Add a new topology module under
   [`topology/`](./isolab/src/topology).
2. Keep `Lab` generic and move only the concrete addressing/routing rules into
   the new topology module.
3. Point the relevant scenario at that topology instead of editing `Lab`.

## How To Add New Boards

The intended layering for a future board integration such as BMS is:

1. Add `components/bms.rs`.
2. Implement a minimal emulator that speaks the real wire protocol expected by
   `flight-computer`.
3. Add any board-specific mappings or Servo setup needed by the scenario.
4. Create a new scenario family or extend the existing setup to optionally
   start the BMS component.

Recommended discipline:

- keep board protocol generation out of scenario files
- keep topology rules out of component files
- keep assertions out of component files

## Extensibility Guidelines

When extending the harness, prefer:

- generic `Lab` primitives over hardcoded topology logic
- topology-specific modules over inline namespace setup in scenarios
- component modules over ad hoc helper functions
- client helpers over repeated HTTP/WebSocket logic
- scenario-specific assertions over broad hidden magic

Avoid:

- teaching `Lab` about specific boards
- adding more hardcoded services directly into `lib.rs`
- reintroducing a single-file orchestrator
- mixing protocol generation, process startup, and assertions in one function
