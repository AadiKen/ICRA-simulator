# Architecture confirmation

The production flow is:

`environmental sources -> source adapters -> environment registry/resolution -> vehicle + sensor + environment + task assembly -> Fossen-style dynamics core -> scalar Node or tensor backend -> Python/RL interface`

The MCP server attaches at the scenario/tool boundary. It exposes discovery,
validation, construction, execution, and artifact/metric retrieval; it does not
replace the dynamics backend.

Relevant implementation locations include `packages/environment`,
`packages/vehicle-sdk`, `packages/sensor-sdk`, `packages/core`, tensor backend
packages, `packages/python-client`, and `packages/mcp-server`.
