# RadioCore

Central radio management system providing CAT command processing, state management, and multi-client routing.

## Overview

RadioCore contains the core radio control logic:

- **RadioManager** - Central coordinator for state, command dispatch, and serial I/O
- **CATHandler** - Parses CAT frames and routes to CommandDispatcher
- **CommandDispatcher** - Routes commands to appropriate handlers based on prefix
- **ForwardingPolicy** - Determines which responses to forward to which interfaces

## Architecture

```
USB CDC ────┐
            │
TCP/IP  ────┐
            ├─► CATHandler(local) ──► CommandDispatcher ──► CommandHandlers
Radio(RRC) ─┤                              │                      │
            ├─► CATHandler(remote)         │                      │
Display ────┘                              ▼                      ▼
                                    RadioState ◄────── State Updates
```
RRC in this case is the RemoteRig but it could very well be the real deal. 

## Key Features

- **Multi-source routing** - USB CDC0, CDC1, TCP, Display, Panel, Radio, Macro
- **Query-answer pairing** - Routes radio responses to originating interface
- **TTL caching** - Serves cached responses for recent queries
- **TX ownership** - Mutual exclusion with timeout protection
- **Control lease** - Priority-based arbitration for multi-client scenarios

## Configuration

Kconfig options for:
- Polling intervals (normal/burst modes)
- AI mode behavior
- Transverter offset settings

## Dependencies

- StateManager (RadioState)
- CatParser
- CommandHandlers
- ISerialChannel
- CommonConstants