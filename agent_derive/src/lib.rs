extern crate proc_macro;

use proc_macro::TokenStream;
use quote::quote;
use syn;

#[proc_macro_derive(AgentDerive)]
pub fn agent_derive(input: TokenStream) -> TokenStream {
  let ast = syn::parse(input).unwrap();
  impl_agent_derive(&ast)
}

fn impl_agent_derive(ast: &syn::DeriveInput) -> TokenStream {
  let name = &ast.ident;
  let gen = quote! {
    impl #name {
      pub fn new(landmarks: Vec<Point>) -> #name {
        let actual = na::Vector3::new(0.0, 0.0, 0.0);
        let observed: Vec<Observed> = Vec::new();
        #name { landmarks, actual, observed }
      }
    }

    impl AgentDerive for #name {
      fn get_name(&self) -> &str {
        stringify!(#name)
      }
      fn get_landmarks(&self) -> &Vec<Point> {
        &self.landmarks
      }
      fn set_actual(&mut self, actual: na::Vector3<f64>) -> () {
        self.actual = actual;
      }
      fn get_actual(&self) -> &na::Vector3<f64> {
        &self.actual
      }
      fn set_observed(&mut self, observed: Vec<Observed>) -> () {
        self.observed = observed;
      }
      fn get_observed(&self) -> &Vec<Observed> {
        &self.observed
      }
    }
  };
  gen.into()
}