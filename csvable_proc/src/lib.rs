use proc_macro2::{ Ident, Span, TokenStream };
use quote::quote;

// I love macro implemnetation
fn impl_csvable(ast: &syn::DeriveInput) -> proc_macro::TokenStream {
    let name = &ast.ident;
    match &ast.data {
      syn::Data::Struct(data) => {
        match &data.fields {
          syn::Fields::Named(fields) => {
            let mut fields_recurse = TokenStream::new();
            let mut content_recurse = TokenStream::new();
            for field in &fields.named {
              // I don't care
              let field_name = field.ident.clone().unwrap_or(Ident::new("unknown", Span::call_site()));
              if field.attrs.iter().any(|attr| attr.path().is_ident("csv_skip")) {
                continue;
              }
              // this is horrible, but I don't care to do a cleaner implementation
              let generated = quote! {
                new_prefix = String::from(prefix);
                if new_prefix.len() > 0 {
                  new_prefix.push('_');
                }
                new_prefix.push_str(stringify!(#field_name));
                output.extend(self.#field_name.to_header(new_prefix.as_str()));
              };
              fields_recurse.extend(generated);
              let generated_content = quote! {
                output.extend(self.#field_name.to_content());
              };
              content_recurse.extend(generated_content);
            }
            let generated = quote! {
              impl CSVable for #name {
                fn to_header(&self, prefix : &str) -> Vec<String> {
                  let mut new_prefix = String::from(prefix);
                  
                  let mut output = Vec::<String>::new();

                  #fields_recurse;

                  output
                }

                fn to_content(&self) -> Vec<String> {
                  let mut output = Vec::<String>::new();

                  #content_recurse

                  output
                }
              }
            };
            
            generated
          },
          _ => TokenStream::new()
        }
      }
      _ => TokenStream::new()
    }.into() // convert to proc_macro instead of proc_macro2
    
}

#[proc_macro_derive(CSVable, attributes(csv_skip))]
pub fn csvable_derive(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
  // Construct a representation of Rust code as a syntax tree
  // that we can manipulate.
  let ast = syn::parse(input).unwrap();

  // Build the trait implementation.
  impl_csvable(&ast)
}