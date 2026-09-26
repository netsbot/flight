use flight_core::comms::Command;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(typescript_custom_section)]
const TS_IMPORT: &'static str = r#"import type { Command } from "../types/Command";"#;

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(typescript_type = "Command")]
    pub type JsCommand;
}

#[wasm_bindgen]
pub fn decode_packet(bytes: &[u8]) -> Result<JsCommand, JsError> {
    let mut buf = bytes.to_vec();
    let cmd =
        postcard::from_bytes_cobs::<Command>(&mut buf).map_err(|e| JsError::new(&e.to_string()))?;
    Ok(serde_wasm_bindgen::to_value(&cmd)
        .map_err(|e| JsError::new(&e.to_string()))?
        .unchecked_into())
}

#[wasm_bindgen]
pub fn encode_packet(command: JsCommand) -> Result<Vec<u8>, JsError> {
    let cmd: Command = serde_wasm_bindgen::from_value(command.into())
        .map_err(|e| JsError::new(&e.to_string()))?;
    postcard::to_allocvec_cobs(&cmd).map_err(|e| JsError::new(&e.to_string()))
}
