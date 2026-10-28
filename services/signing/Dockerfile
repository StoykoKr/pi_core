# Signing server launcher — server.py itself is UNCHANGED trusted legacy code
# (contract §10: containerized launchers only). Cert dir comes in as a rw
# volume; RE4LIFE_CERT_DIR is set by compose.
FROM python:3.12-slim

RUN pip install --no-cache-dir fastapi "uvicorn[standard]" cryptography pydantic

WORKDIR /app
COPY server.py .

CMD ["python3", "server.py"]
