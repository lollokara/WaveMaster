#!/usr/bin/env python3
"""
Loopback relay for the WaveMaster WiFi link.

Rayforge on macOS can fail to reach the controller with "No route to host
(os error 65)" even with the Local Network switch on (macOS keeps discarding
its cached permission for an ad-hoc signed app). Connections to 127.0.0.1 are
not covered by that rule, so this script, run from a terminal that does have
the permission, forwards 127.0.0.1:<local port> to the controller.

Usage:
    python3 tools/tcp_relay.py 192.168.3.51            # listens on 127.0.0.1:2323
    python3 tools/tcp_relay.py 192.168.3.51 --listen 2323 --port 23

Then set the Rayforge machine's driver arguments to host 127.0.0.1, port 2323.
Keep the script running while you use Rayforge. It accepts one client at a time
(the controller takes one too) and closes the controller side when Rayforge
disconnects, which the firmware treats like a lost host: job stopped, laser off.
"""
import argparse
import asyncio


async def pump(r, w):
    try:
        while True:
            data = await r.read(1024)
            if not data:
                break
            w.write(data)
            await w.drain()
    except (ConnectionError, asyncio.CancelledError):
        pass
    finally:
        w.close()


async def handle(cr, cw, host, port):
    try:
        ur, uw = await asyncio.wait_for(asyncio.open_connection(host, port), 5)
    except (OSError, asyncio.TimeoutError) as e:
        print('controller unreachable: %s' % e)
        cw.close()
        return
    print('client connected')
    t1 = asyncio.create_task(pump(cr, uw))
    t2 = asyncio.create_task(pump(ur, cw))
    await asyncio.wait({t1, t2}, return_when=asyncio.FIRST_COMPLETED)
    for t in (t1, t2):
        t.cancel()
    uw.close()
    cw.close()
    print('client disconnected')


async def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('host', help='controller IP address')
    ap.add_argument('--port', type=int, default=23, help='controller port (default 23)')
    ap.add_argument('--listen', type=int, default=2323, help='local port (default 2323)')
    a = ap.parse_args()
    srv = await asyncio.start_server(lambda r, w: handle(r, w, a.host, a.port), '127.0.0.1', a.listen)
    print('relaying 127.0.0.1:%d -> %s:%d' % (a.listen, a.host, a.port))
    async with srv:
        await srv.serve_forever()


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
