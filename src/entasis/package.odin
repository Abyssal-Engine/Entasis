// package entasis provides the stable integration API for Entasis
//
// the entasis_physics and entasis_utilities packages execute the work.
// this package does not duplicate simulation state or add a global registry,
// implicit locks, or hidden background work. resource ownership is explicit
// throughout each resource's lifetime. allocation follows the selected policies
package entasis
