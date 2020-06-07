pub enum USBError {
    DescriptorNotAvailable,
    ControlEndpointTimeout,
    CommandTimeout,
    InvalidArgument,
    Other
}