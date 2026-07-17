#!/usr/bin/env python3
# Scripted verify of the MCP server, run inside the container with the flight stack up
import asyncio

from fastmcp import Client
from fastmcp.client.transports import StdioTransport

EXPECTED_TOOL_NAMES = {
    'takeoff', 'goto', 'land', 'abort', 'get_telemetry', 'get_mission_state'}


# Launch the MCP server over stdio and run the scripted checks
async def main():
    transport = StdioTransport(
        command='bash',
        args=['-c',
              'source /opt/ros/jazzy/setup.bash && '
              'source /ws/install/setup.bash && ros2 run kestrel mcp_server'])

    async with Client(transport) as client:
        tools = await client.list_tools()
        tool_names = {tool.name for tool in tools}
        assert EXPECTED_TOOL_NAMES.issubset(tool_names), tool_names
        print(f'tools: {sorted(tool_names)}')

        telemetry = await client.call_tool('get_telemetry')
        print(f'telemetry before takeoff: {telemetry.data}')

        takeoff_result = await client.call_tool('takeoff', {'altitude': 3.0})
        print(f'takeoff: {takeoff_result.data}')

        telemetry_after_takeoff = await client.call_tool('get_telemetry')
        print(f'telemetry after takeoff: {telemetry_after_takeoff.data}')

        land_result = await client.call_tool('land')
        print(f'land: {land_result.data}')


if __name__ == '__main__':
    asyncio.run(main())
