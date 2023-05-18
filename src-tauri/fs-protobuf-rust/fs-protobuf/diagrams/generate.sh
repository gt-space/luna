protodot -src ../common.proto -output common
protodot -src ../command.proto -output command
protodot -src ../core.proto -output core
protodot -src ../data.proto -output data
protodot -src ../device.proto -output device
protodot -src ../log.proto -output log
protodot -src ../status.proto -output status
protodot -src ../procedure.proto -output procedure
cp ~/protodot/generated/*.svg ./