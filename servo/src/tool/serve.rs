use crate::{
  interface,
  server::{flight, Server},
};
use clap::ArgMatches;
use std::io;
use std::path::Path;

/// Function used to convert std::future::pending to a join handle in the serve
/// functions selection of a shutdown task.
///
/// In theory this can be converted to checking a value in shared every X unit
/// of time if we wish to allow the GUI to shut down servo and thus the command
/// simple set the value to quit / true / etc. (Potential future feature)
async fn infinite_hang() -> io::Result<()> {
  std::future::pending::<()>().await;
  Ok(())
}

/// Performs the necessary setup to connect to the servo server.
///
/// This function initializes database connections, spawns background tasks,
/// and starts the TUI and the HTTP server to serve the application upon
/// request. It also configures the HTTP server to gracefully shut down if the
/// TUI terminates outside of quiet mode.
pub fn serve(servo_dir: &Path, args: &ArgMatches) -> anyhow::Result<()> {
  let volatile = args.get_one::<bool>("volatile").copied().unwrap_or(false);

  let quiet = args.get_one::<bool>("quiet").copied().unwrap_or(false);

  let database_path = servo_dir.join("database.sqlite");
  let server = Server::new((!volatile).then_some(&database_path))?;

  server.shared.database.migrate()?;

  tokio::runtime::Builder::new_multi_thread()
    .worker_threads(10)
    .enable_all()
    .build()
    .unwrap()
    .block_on(async move {
      tokio::spawn(flight::auto_connect(&server.shared));
      tokio::spawn(flight::receive_vehicle_state(&server.shared));
      tokio::spawn(server.shared.database.log_vehicle_state(&server.shared));

      // The task that, once finished, will signal the server to terminate.
      // Set to the TUI if it is launched, otherwise set to an infinitely
      // hanging await that should(?) consume no resources.
      // let shutdown_task: tokio::task::JoinHandle<io::Result<()>>;
      let shutdown_task = if !quiet {
        tokio::spawn(interface::display(server.shared.clone()))
      } else {
        tokio::spawn(infinite_hang())
      };

      server.serve(shutdown_task).await
    })?;

  Ok(())
}
