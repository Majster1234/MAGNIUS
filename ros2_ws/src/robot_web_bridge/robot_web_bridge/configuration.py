"""Asynchroniczny klient węzła konfiguracji; backend nie zapisuje własnych kopii plików."""
import asyncio
import json


class ConfigurationClient:
    def __init__(self, node):
        from robot_control_interfaces.srv import ConfigurationCommand
        self.kind = ConfigurationCommand
        self.client = node.create_client(ConfigurationCommand, '/robot/configuration/command')
        self.pending = 0

    async def __call__(self, operation, **values):
        if not self.client.service_is_ready():
            return 503, {'detail': 'Węzeł konfiguracji ROS jest niedostępny'}
        if self.pending >= 4:
            return 503, {'detail': 'Węzeł konfiguracji obsługuje inne żądania'}
        request = self.kind.Request(operation=operation, request_json=json.dumps(values, ensure_ascii=False, allow_nan=False))
        self.pending += 1
        future = self.client.call_async(request)
        loop = asyncio.get_running_loop(); result = loop.create_future()

        def deliver(done):
            def finish():
                if result.done():
                    return
                try:
                    result.set_result(done.result())
                except Exception as error:
                    result.set_exception(error)
            loop.call_soon_threadsafe(finish)
        future.add_done_callback(deliver)
        try:
            response = await asyncio.wait_for(result, 2)
            return response.status_code, json.loads(response.response_json)
        except Exception:
            return 503, {'detail': 'Brak odpowiedzi konfiguracji; odczytaj stan przed ponowieniem zapisu'}
        finally:
            self.pending -= 1
            self.client.remove_pending_request(future)
            if not future.done():
                future.cancel()
