FROM ubuntu:22.04

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ca-certificates \
        libudev1 \
        python3.10 \
        python3.10-venv \
    && rm -rf /var/lib/apt/lists/* \
    && python3.10 -m venv /opt/venv

ENV PATH="/opt/venv/bin:${PATH}" \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

RUN pip install --no-cache-dir wuji-sdk

RUN groupadd --gid 1000 wuji \
    && useradd --uid 1000 --gid wuji --create-home --shell /bin/bash wuji \
    && mkdir -p /workspace /home/wuji/.wuji \
    && chown wuji:wuji /workspace /home/wuji/.wuji

USER wuji:wuji
WORKDIR /workspace
CMD ["bash"]
