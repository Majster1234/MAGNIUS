"""Ostatni opis modelu z ROS; bez dostępu do dowolnych plików lub adresów URL."""
from hashlib import sha256
import threading
import xml.etree.ElementTree as ET


class DescriptionStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.model = None

    def update(self, message):
        text = message.data
        model = None
        try:
            if len(text.encode('utf-8')) > 262144:
                raise ValueError('Zbyt duży model')
            root = ET.fromstring(text)
            if root.tag != 'robot' or root.findall('.//mesh') or root.findall('.//texture'):
                raise ValueError('Podgląd obsługuje bryły URDF')
            expected = {f'joint_{i}' for i in range(1, 7)}
            actual = {j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed'}
            if actual != expected:
                raise ValueError('Nieprawidłowe osie modelu')
            model = (text, sha256(text.encode()).hexdigest())
        except (ET.ParseError, ValueError, TypeError):
            pass
        with self.lock:
            self.model = model

    def snapshot(self):
        with self.lock:
            return self.model
