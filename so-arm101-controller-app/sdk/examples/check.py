"""
연결 확인. 6축 다 응답하는지 한눈에 본다.

실행:  python3 check.py
"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from soarm import Robot

TEAM = "http://localhost:8000"   # 다른 PC 면 "team1" 또는 IP

for jid, ang in Robot(TEAM).angles().items():
    print(f"  {'✓' if ang is not None else '✗'} joint {jid}: {ang}")
