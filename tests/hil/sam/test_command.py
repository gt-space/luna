import pytest
from sam import sam_command as sam_cmd

def test_valves_and_gpio():
    # Turn off all valves
    sam_cmd.safe_valves_py()
    num_valves = sam_cmd.get_num_valves()

    # Check that all valves are off
    for valve in range(1, num_valves + 1):  # start at 1
        assert sam_cmd.get_valve_state(valve) is False
    
    # Power a valve
    sam_cmd.execute_valve_py(1, True)
    
    # Check that valve 1 is powered
    assert sam_cmd.get_valve_state(1) is True
    
    # # Turn off all valves
    # sam_cmd.safe_valves_py()

    # # Initialize GPIO pins
    # sam_cmd.init_gpio_py()

    # # Power a valve
    # sam_cmd.execute_valve_py(1, True)

    # # Reset valve current selection pins
    # sam_cmd.reset_valve_current_sel_pins_py()
    
    # sam_cmd.execute_valve_py(1, True)
    # pin = sam_cmd.GPIO_CONTROLLERS[0].get_pin(8)
    # assert pin.value == True
    # print(sam_cmd.get_gpio_controllers_len())
    # len = sam_command.get_gpio_controllers_len()
    # ctrl = sam_command.get_gpio_controller(0)
    # print(len, ctrl)