#[tokio::main]
async fn main() -> anyhow::Result<()> {
  isolab::run().await
}
