pub trait CSVable {
  fn to_header(&self, prefix : &str) -> Vec<String>;
  fn to_content(&self) -> Vec<String>;
}

impl CSVable for f64 {
  fn to_header(&self, prefix : &str) -> Vec<String> {
    vec![String::from(prefix)]
  }
  fn to_content(&self) -> Vec<String> {
    vec![format!("{:.3}", self)]
  }
}

impl CSVable for bool {
  fn to_header(&self, prefix : &str) -> Vec<String> {
    vec![String::from(prefix)]
  }
  fn to_content(&self) -> Vec<String> {
    vec![format!("{}", self)]
  }
}

impl CSVable for String {
  fn to_header(&self, prefix : &str) -> Vec<String> {
    vec![String::from(prefix)]
  }
  fn to_content(&self) -> Vec<String> {
    vec![self.clone()]
  }
}