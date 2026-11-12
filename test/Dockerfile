FROM python:3.11-slim-bookworm

RUN apt-get update \
    && apt-get install -y --no-install-recommends iverilog make \
    && apt-get clean

WORKDIR /work
COPY test/requirements.txt /tmp/test-requirements.txt
RUN pip install --no-cache-dir -r /tmp/test-requirements.txt \
    && useradd --create-home --uid 1000 runner

COPY --chown=runner:runner . /work
USER runner
ENTRYPOINT ["python", "scripts/run_tests.py"]
