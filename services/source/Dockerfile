FROM python:3.11-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
        libusb-1.0-0 \
        libglib2.0-0 \
        && rm -rf /var/lib/apt/lists/*

COPY services/source/requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

COPY common /opt/edgebot
RUN pip install --no-cache-dir -e /opt/edgebot

WORKDIR /app
COPY services/source/ .

CMD ["python", "source.py"]
