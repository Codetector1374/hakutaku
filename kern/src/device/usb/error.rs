
#[derive(Debug, Copy, Clone)]
pub enum USBError {
    DescriptorNotAvailable,
    ControlEndpointTimeout,
    CommandTimeout,
    InvalidArgument,

    // Response Errors
    InvalidDescriptor,

    Other
}