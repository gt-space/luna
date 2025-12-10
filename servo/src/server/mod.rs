/// Server database components.
pub mod database;

/// Server error components.
pub mod error;

/// Flight-related components such as the `FlightComputer` struct.
pub mod flight;

/// All server API route functions.
pub mod routes;

use axum::Router;
use common::comm::VehicleState;
pub use database::Database;
pub use error::{ServerError as Error, ServerResult as Result};
pub use flight::FlightComputer;
use tokio::time::Instant;
use tower_http::cors::{self, CorsLayer};

use std::{io, net::SocketAddr, path::Path, sync::Arc};
use tokio::{
  net::TcpListener,
  sync::{Mutex, Notify},
  task::JoinHandle,
};

/// Contains all of Servo's shared server state.
/// 
/// The number of mutexes here for statistics is getting absurd.
/// We should consolidate them soon
#[derive(Clone, Debug)]
pub struct Shared {
  /// The database, a wrapper over `Arc<Mutex<SqlConnection>>`, so that it may
  /// be accessed in route functions.
  pub database: Database,

  /// The option for a flight computer.
  pub flight: Arc<(Mutex<Option<FlightComputer>>, Notify)>,

  /// The option for a ground computer.
  pub ground: Arc<(Mutex<Option<FlightComputer>>, Notify)>,

  /// The state of the vehicle, including both flight and ground components.
  pub vehicle: Arc<(Mutex<VehicleState>, Notify)>,

  /// keep track of the last time the vehicle state was updated
  pub last_vehicle_state: Arc<(Mutex<Option<Instant>>, Notify)>,

  /// keep track of the last time the vehicle state was updated
  pub last_tel_vehicle_state: Arc<(Mutex<Option<Instant>>, Notify)>,

  /// keep track of the update rate / rolling duration of the vehicle state
  pub rolling_duration: Arc<(Mutex<Option<f64>>, Notify)>,
  
  /// keep track of the update rate / rolling duration of the vehicle state
  pub rolling_tel_duration: Arc<(Mutex<Option<f64>>, Notify)>,

  /// Number of packets received (not tel)
  pub packet_count : Arc<(Mutex<usize>, Notify)>,

  /// Number of packets received (tel)
  pub tel_packet_count : Arc<(Mutex<usize>, Notify)>,
}

/// The server, constructed with all route functions ready.
#[derive(Clone, Debug)]
pub struct Server {
  /// The shared state of the server, to be passed to route functions.
  pub shared: Shared,
}

async fn wait_for_display_end(shutdown_future: JoinHandle<io::Result<()>>) {
  let _ = shutdown_future.await;
}

impl Server {
  /// Constructs a new `Server` and opens a `Database` based on the path given.
  pub fn new(database_path: Option<&Path>) -> anyhow::Result<Self> {
    let database;

    if let Some(path) = database_path {
      database = Database::open(path)?;
    } else {
      database = Database::volatile()?;
    }

    let shared = Shared {
      database,
      flight: Arc::new((Mutex::new(None), Notify::new())),
      ground: Arc::new((Mutex::new(None), Notify::new())),
      vehicle: Arc::new((Mutex::new(VehicleState::new()), Notify::new())),
      last_vehicle_state: Arc::new((Mutex::new(None), Notify::new())),
      rolling_duration: Arc::new((Mutex::new(None), Notify::new())),
      last_tel_vehicle_state: Arc::new((Mutex::new(None), Notify::new())),
      rolling_tel_duration: Arc::new((Mutex::new(None), Notify::new())),
      packet_count: Arc::new((Mutex::new(0), Notify::new())),
      tel_packet_count: Arc::new((Mutex::new(0), Notify::new())),
    };

    Ok(Server { shared })
  }

  /// Serves the route functions with permissive CORS. Exits when the
  /// shutdown_future returns via a graceful shutdown.
  ///
  /// Of note is that this graceful shutdown can wait for outstanding requests
  /// to complete (such as an oversized export), which may delay the time it
  /// takes for the program to truly exit after the shutdown_future has
  /// returned.
  pub async fn serve(
    &self,
    shutdown_future: JoinHandle<io::Result<()>>,
  ) -> io::Result<()> {
    use axum::routing::{delete, get, post, put};

    let cors = CorsLayer::new()
      .allow_methods(cors::Any)
      .allow_headers(cors::Any)
      .allow_origin(cors::Any);

    let router = Router::new()
      .route("/data/forward", get(routes::forward_data))
      .route("/data/export", post(routes::export))
      .route("/admin/sql", post(routes::execute_sql))
      .route("/operator/command", post(routes::dispatch_operator_command))
      .route("/operator/mappings", get(routes::get_mappings))
      .route("/operator/mappings", post(routes::post_mappings))
      .route("/operator/mappings", put(routes::put_mappings))
      .route("/operator/mappings", delete(routes::delete_mappings))
      .route(
        "/operator/active-configuration",
        get(routes::get_active_configuration),
      )
      .route(
        "/operator/active-configuration",
        post(routes::activate_configuration),
      )
      .route("/operator/calibrate", post(routes::calibrate))
      .route("/operator/sequence", get(routes::retrieve_sequences))
      .route("/operator/sequence", put(routes::save_sequence))
      .route("/operator/sequence", delete(routes::delete_sequence))
      .route("/operator/abort-config", get(routes::retrieve_abort_configs))
      .route("/operator/abort-config", put(routes::save_abort_config))
      .route("/operator/abort-config", delete(routes::delete_abort_config))
      .route("/operator/set-stage", put(routes::set_abort_config))
      .route("/operator/run-sequence", post(routes::run_sequence))
      .route("/operator/stop-sequence", post(routes::stop_sequence))
      .route("/operator/abort", post(routes::abort))
      .route("/operator/trigger", get(routes::get_triggers))
      .route("/operator/trigger", put(routes::set_trigger))
      .route("/operator/trigger", delete(routes::delete_trigger))
      .route("/operator/camera", post(routes::enable_camera))
      .route("/operator/arm-lugs", post(routes::arm_lugs))
      .route("/operator/detonate-lugs", post(routes::detonate_lugs))
      .layer(cors)
      .with_state(self.shared.clone())
      .into_make_service_with_connect_info::<SocketAddr>();

    let listener = TcpListener::bind("0.0.0.0:7200").await?;
    axum::serve(listener, router)
      .with_graceful_shutdown(wait_for_display_end(shutdown_future))
      .await?;

    Ok(())
  }
}
