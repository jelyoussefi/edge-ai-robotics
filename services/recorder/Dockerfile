FROM python:3.11-slim

COPY services/recorder/requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

COPY common /opt/edgebot
RUN pip install --no-cache-dir -e /opt/edgebot

WORKDIR /app
COPY services/recorder/ .

CMD ["python", "recorder.py"]
