use flight_core::comms::Message;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(typescript_custom_section)]
const TS_IMPORT: &'static str = r#"import type { Message } from "../types/Message";"#;

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(typescript_type = "Message")]
    pub type JsMessage;
}

#[wasm_bindgen]
pub fn decode_packet(bytes: &[u8]) -> Result<JsMessage, JsError> {
    let mut buf = bytes.to_vec();
    let msg =
        postcard::from_bytes_cobs::<Message>(&mut buf).map_err(|e| JsError::new(&e.to_string()))?;
    Ok(serde_wasm_bindgen::to_value(&msg)
        .map_err(|e| JsError::new(&e.to_string()))?
        .unchecked_into())
}

#[wasm_bindgen]
pub fn encode_packet(message: JsMessage) -> Result<Vec<u8>, JsError> {
    let msg: Message = serde_wasm_bindgen::from_value(message.into())
        .map_err(|e| JsError::new(&e.to_string()))?;
    postcard::to_allocvec_cobs(&msg).map_err(|e| JsError::new(&e.to_string()))
}
