# SITL

Software-in-the-loop testing for Luna using Renode, which emulates hardware: a
microcontroller and the chips around it. Real firmware runs on it unchanged,
and peripheral models (sensors, actuators, converters) stand in for the
physical parts.

## Layout

| Path | What's there |
| --- | --- |
| `peripherals/` | Renode models of individual chips, in C# |
| `platforms/` | Descriptions of emulated boards |
| `tests/peripherals/<Name>/` | Tests for one model on its own |
| `tests/boards/<Name>/` | Firmware running on an emulated board |
| `tests/checks/` | Repository-wide checks |
| `tests/common/` | Shared test keywords |
| `tests.yml` | Which paths each test depends on |

## Adding a peripheral

1. Write the model in `peripherals/`, following the datasheet.
2. Add a test in `tests/peripherals/<Name>/` that drives the model the way a
   driver would and checks what comes back. The existing tests are the best
   starting point.
3. Add an entry for it in `tests.yml`.
4. Run it locally, then open a pull request.

A peripheral test checks the model against your reading of the datasheet, so
take the expected values from the datasheet rather than from what the model
happens to return.

## Running tests locally

Install [Renode](https://renode.io), then point `renode-test` at the test
files, for example:

```bash
renode-test sitl/tests/peripherals/LIS2MDL/*.robot
```

Pass the `.robot` files rather than the folder: given a folder, `renode-test`
finds nothing to run and still reports success.

## In CI

Pull requests run the `SITL Tests` workflow. It works out from `tests.yml`
which tests the change affects and runs only those, along with a check that
every model still compiles. The `sitl` check summarises the result. When a test
fails, the run's artifacts include Renode's logs and a snapshot of the
emulation at the point of failure.
