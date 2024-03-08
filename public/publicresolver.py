import json
import os


def getNetInfo():
    FILE = "public/netinfo.json"
    if os.path.exists(FILE):
        with open(FILE, "r") as f:
            return json.load(f)
    return None
