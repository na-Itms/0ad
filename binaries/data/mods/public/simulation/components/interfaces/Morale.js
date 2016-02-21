Engine.RegisterInterface("Morale");

// Message of the form { "from": 100, "to", 90 },
// sent whenever morale changes.
Engine.RegisterMessageType("MoraleChanged");
