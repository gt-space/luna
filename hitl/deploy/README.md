# HITL CM5 setup

## Once per board

On the CM5, with the Meerkat runner's public key copied over:

```sh
sudo ./setup-cm5.sh --key meerkat.pub
sudo reboot
./setup-cm5.sh --check
```

The script turns on SPI0, SPI1, SPI5 and I2C1, creates the `hitl` user, lets
it restart `hitl.service` and nothing else, and enables the service. Running
it again changes nothing.

## Bring-up

Do these in order. Each step only needs the ones before it.

1. `/opt/hitl/hitl --mock`, then from the Meerkat `hitl --query <cm5>:8380`.
   This checks the network and the service with no chips involved.
2. Each chip's example on its own (`cargo run -p <chip> --example ...` built
   for aarch64), with a meter or scope on the output.
3. `sudo systemctl restart hitl.service` and `journalctl -u hitl -f`. Every
   chip should log "ready". A chip that fails is logged and the rest keep
   working.
4. From the Meerkat, set a DAC channel and read it back on the ADC. Stop the
   client and check the DAC drops to 0 V within a second.

## CI

`.github/workflows/hitl.yml` builds the service, copies it to the board,
restarts it, and waits until `hitl --query` reports the commit under test. It
only runs when the repo variable `HITL_RUNNER` is `true`, and never for pull
requests from forks.
