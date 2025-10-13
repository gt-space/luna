use std::io::{self, Write};
use once_cell::sync::Lazy;
use common::comm::gpio::{Gpio, PinMode, PinValue};

static GPIO0: Lazy<Gpio> = Lazy::new(|| Gpio::open_controller(0));
static GPIO1: Lazy<Gpio> = Lazy::new(|| Gpio::open_controller(1));
static GPIO2: Lazy<Gpio> = Lazy::new(|| Gpio::open_controller(2));
static GPIO3: Lazy<Gpio> = Lazy::new(|| Gpio::open_controller(3));

fn read_line(prompt: &str) -> io::Result<String> {
    print!("{prompt}");
    io::stdout().flush()?;
    let mut s = String::new();
    io::stdin().read_line(&mut s)?;
    Ok(s.trim().to_string())
}

fn parse_usize_in_range(s: &str, lo: usize, hi: usize) -> Option<usize> {
    s.parse::<usize>().ok().filter(|v| (*v >= lo) && (*v <= hi))
}

fn main() -> io::Result<()> {
    // run as root
    let gpios: [&'static Gpio; 4] = [&*GPIO0, &*GPIO1, &*GPIO2, &*GPIO3];

    println!("GPIO single-pin tester");
    println!("Flow: pick (controller, bit) → commands: s=toggle, h=HIGH, l=LOW, r=read, n=new pin, q=quit\n");

    'outer: loop {
        // controller and pin select
        let ctrl = loop {
            let s = read_line("Controller/bank (0..3): ")?;
            if let Some(v) = parse_usize_in_range(&s, 0, 3) { break v; }
            eprintln!("Please enter 0..3.");
        };

        let bit = loop {
            let s = read_line("Pin/bit within controller (0..31): ")?;
            if let Some(v) = parse_usize_in_range(&s, 0, 31) { break v; }
            eprintln!("Please enter 0..31.");
        };

        // configure once as output, default low 
        
        let mut p = gpios[ctrl].get_pin(bit);
        p.mode(PinMode::Output);
        p.digital_write(PinValue::Low);

        let mut high = false;

        println!(
            "\nControlling GPIO{ctrl}_{bit}. Commands: s=toggle, h=HIGH, l=LOW, r=read, n=new pin, q=quit"
        );

        loop {
            let cmd = read_line("> ")?;
            let ch = cmd.chars().find(|c| !c.is_whitespace());

            match ch {
                Some('q') | Some('Q') => {
                    // drive low on exit
                    let mut p = gpios[ctrl].get_pin(bit);
                    p.digital_write(PinValue::Low);
                    println!("Goodbye.");
                    break 'outer;
                }
                Some('n') | Some('N') => {
                    // drive low and go pick a new pin
                    let mut p = gpios[ctrl].get_pin(bit);
                    p.digital_write(PinValue::Low);
                    println!("Returning to pin selection…\n");
                    break; 
                }
                Some('s') | Some('S') => {
                    high = !high;
                    let mut p = gpios[ctrl].get_pin(bit);
                    p.digital_write(if high { PinValue::High } else { PinValue::Low });
                    println!("GPIO{ctrl}_{bit} -> {}", if high { "HIGH" } else { "LOW" });
                }
                Some('h') | Some('H') => {
                    high = true;
                    let mut p = gpios[ctrl].get_pin(bit);
                    p.digital_write(PinValue::High);
                    println!("GPIO{ctrl}_{bit} -> HIGH");
                }
                Some('l') | Some('L') => {
                    high = false;
                    let mut p = gpios[ctrl].get_pin(bit);
                    p.digital_write(PinValue::Low);
                    println!("GPIO{ctrl}_{bit} -> LOW");
                }
                Some('r') | Some('R') => {
                    let p = gpios[ctrl].get_pin(bit);
                    let level = p.digital_read();
                    println!("GPIO{ctrl}_{bit} reads {:?}", level);
                }
                _ => {
                    println!("Commands: s=toggle, h=HIGH, l=LOW, r=read, n=new pin, q=quit");
                }
            }
        }
    }

    Ok(())
}
