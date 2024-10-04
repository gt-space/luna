use common::comm::BmsControlMessage;
use gpio;

pub fn begin(gpio_controllers: Vec<Arc<Gpio>>) {
    let socket = UdpSocket::bind("0.0.0.0:8378").expect("Cannot bind to socket");
    let mut buf = [0; 65536];
    loop {
      let (num_bytes, _src_addr) =
        socket.recv_from(&mut buf).expect("no data received");
      println!("{:?}", num_bytes);
      let deserialized_result =
        postcard::from_bytes::<BmsControlMessage>(&buf[..num_bytes]);
      println!("{:#?}", deserialized_result);
      match deserialized_result {
        Ok(message) => {
          execute(message, gpio_controllers.clone());
        }
        Err(_error) => fail!("Bad command message from flight computer"),
      };
    }
}

pub fn enable_battery_power {

}

pub fn disable_battery_power {

}

pub fn sam_enable {

}

pub fn sam_disable {

}

pub fn estop_reset {

}

pub fn adc_select {

}

pub fn reco_enable {

}