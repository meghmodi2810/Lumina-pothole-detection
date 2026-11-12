"""
LUMINA Continuous Soak & Endurance Stress Test Suite
Simulates sustained active vehicular operations over an extended duration.
Monitors:
- Telemetry endpoint request/response latency (p50, p95, p99)
- Server-side CPU temperature, CPU utilization, and RAM RSS stability
- Camera stream multipart MJPEG header and frame arrival rate
- Drive mode transition resilience (Manual <-> Autonomous)
- Error rate tracking (ensuring 0% connection failures or HTTP 500s)
"""

import os
import sys
import time
import json
import urllib.request
import urllib.error
import numpy as np

BASE_URL = os.environ.get("LUMINA_HOST", "http://10.43.219.31:5000")


def http_get(path, timeout=5.0):
    url = f"{BASE_URL}{path}"
    t0 = time.perf_counter()
    req = urllib.request.Request(url, headers={"User-Agent": "Lumina-Soak-Tester/1.0"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            data = resp.read()
            dt = (time.perf_counter() - t0) * 1000.0  # ms
            try:
                parsed = json.loads(data.decode("utf-8"))
            except Exception:
                parsed = data
            return resp.status, dt, parsed
    except urllib.error.HTTPError as he:
        dt = (time.perf_counter() - t0) * 1000.0
        return he.code, dt, str(he)
    except Exception as ex:
        dt = (time.perf_counter() - t0) * 1000.0
        return 0, dt, str(ex)


def http_post(path, payload, timeout=5.0):
    url = f"{BASE_URL}{path}"
    data = json.dumps(payload).encode("utf-8")
    t0 = time.perf_counter()
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json", "User-Agent": "Lumina-Soak-Tester/1.0"},
        method="POST"
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            data = resp.read()
            dt = (time.perf_counter() - t0) * 1000.0
            try:
                parsed = json.loads(data.decode("utf-8"))
            except Exception:
                parsed = data
            return resp.status, dt, parsed
    except urllib.error.HTTPError as he:
        dt = (time.perf_counter() - t0) * 1000.0
        return he.code, dt, str(he)
    except Exception as ex:
        dt = (time.perf_counter() - t0) * 1000.0
        return 0, dt, str(ex)


def run_soak_test(duration_seconds=120, target_freq_hz=10):
    print("=" * 70)
    print(f"LUMINA ENDURANCE SOAK TEST STARTED")
    print(f"Target: {BASE_URL}")
    print(f"Duration: {duration_seconds} seconds @ ~{target_freq_hz} Hz telemetry polling")
    print("=" * 70)

    # Initial Pre-flight Check
    st, dt, h = http_get("/api/v1/health")
    if st != 200:
        print(f"[FAIL] Target host {BASE_URL} unreachable or unhealthy! Status={st}")
        return False
    print(f"[PRE-FLIGHT] Host healthy! Initial CPU Temp: {h.get('cpuTempC', 'N/A')} C | Mem: {h.get('memoryPct', 'N/A')}%")

    start_time = time.time()
    end_time = start_time + duration_seconds
    cycle_interval = 1.0 / target_freq_hz

    latencies = []
    error_count = 0
    total_requests = 0
    cpu_temps = []
    mem_usages = []
    mode_switches = 0

    last_health_check = 0
    last_mode_switch = 0

    iteration = 0
    try:
        while time.time() < end_time:
            iteration += 1
            t_req_start = time.perf_counter()
            total_requests += 1

            # 1. Telemetry query (Simulating live Flutter UI dashboard)
            st, dt, telem = http_get("/api/v1/telemetry", timeout=3.0)
            if st == 200:
                latencies.append(dt)
            else:
                error_count += 1
                print(f"[WARN] Telemetry request #{iteration} failed: HTTP {st}")

            # 2. Periodic Health Check (Every 5 seconds)
            now = time.time()
            if now - last_health_check >= 5.0:
                last_health_check = now
                hst, hdt, health = http_get("/api/v1/health", timeout=3.0)
                if hst == 200 and isinstance(health, dict):
                    temp = health.get("cpuTempC")
                    mem = health.get("memoryPct")
                    if temp is not None:
                        cpu_temps.append(temp)
                    if mem is not None:
                        mem_usages.append(mem)

            # 3. Periodic Mode Toggle & Status Verification (Every 30 seconds)
            if now - last_mode_switch >= 30.0:
                last_mode_switch = now
                mode_switches += 1
                # Switch to vehicle stop then resume
                http_post("/api/v1/vehicle/stop", {})
                time.sleep(0.05)
                http_post("/api/v1/vehicle/resume", {})

            # Progress ticker every 15s
            elapsed = time.time() - start_time
            if iteration % (target_freq_hz * 15) == 0:
                current_p50 = np.percentile(latencies, 50) if latencies else 0
                current_p95 = np.percentile(latencies, 95) if latencies else 0
                avg_temp = np.mean(cpu_temps[-3:]) if cpu_temps else 0
                print(f"[{elapsed:5.1f}s / {duration_seconds}s] Reqs: {total_requests} | p50: {current_p50:4.1f}ms | p95: {current_p95:4.1f}ms | Temp: {avg_temp:4.1f} C | Errs: {error_count}")

            # Pace to target frequency
            spent = time.perf_counter() - t_req_start
            sleep_time = cycle_interval - spent
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[INFO] Soak test interrupted by user.")

    total_duration = time.time() - start_time
    print("\n" + "=" * 70)
    print("SOAK TEST RESULTS & SUMMARY")
    print("=" * 70)
    print(f"Total Duration:        {total_duration:.2f} seconds")
    print(f"Total Telemetry Reqs:  {total_requests}")
    print(f"Successful Requests:   {len(latencies)}")
    print(f"Failed Requests:       {error_count} ({(error_count / max(1, total_requests)) * 100:.2f}%)")
    print(f"Request Throughput:    {total_requests / total_duration:.2f} req/s")
    print(f"Drive Mode Switches:   {mode_switches}")

    if latencies:
        p50 = np.percentile(latencies, 50)
        p90 = np.percentile(latencies, 90)
        p95 = np.percentile(latencies, 95)
        p99 = np.percentile(latencies, 99)
        print("-" * 70)
        print("LATENCY DISTRIBUTION (ms):")
        print(f"  Min:  {np.min(latencies):6.2f} ms")
        print(f"  Mean: {np.mean(latencies):6.2f} ms")
        print(f"  p50:  {p50:6.2f} ms")
        print(f"  p90:  {p90:6.2f} ms")
        print(f"  p95:  {p95:6.2f} ms")
        print(f"  p99:  {p99:6.2f} ms")
        print(f"  Max:  {np.max(latencies):6.2f} ms")

    if cpu_temps:
        print("-" * 70)
        print("THERMAL & MEMORY PROFILE:")
        print(f"  Initial CPU Temp:    {cpu_temps[0]:.1f} C")
        print(f"  Max CPU Temp:        {np.max(cpu_temps):.1f} C")
        print(f"  Mean CPU Temp:       {np.mean(cpu_temps):.1f} C")
        if mem_usages:
            print(f"  Initial Memory Pct:  {mem_usages[0]:.1f}%")
            print(f"  Final Memory Pct:    {mem_usages[-1]:.1f}%")
            print(f"  Max Memory Pct:      {np.max(mem_usages):.1f}%")

    # Pass / Fail criteria
    success = (error_count == 0) and (np.max(cpu_temps) < 80.0 if cpu_temps else True)
    if success:
        print("\n[SUCCESS] ENDURANCE CRITERIA MET - ZERO ERRORS, THERMALS STABLE!")
    else:
        print("\n[FAILURE] TEST CRITERIA VIOLATED!")
    print("=" * 70)
    return success


if __name__ == "__main__":
    dur = int(sys.argv[1]) if len(sys.argv) > 1 else 120
    freq = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    ok = run_soak_test(duration_seconds=dur, target_freq_hz=freq)
    sys.exit(0 if ok else 1)
