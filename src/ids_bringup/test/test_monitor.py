from ids_bringup.monitor import RunMonitor


def feed(monitor, start, end, step=0.01):
    t = start
    done = False
    while t <= end + 1e-9:
        done = monitor.on_reference(t)
        t += step
    return done


def test_run_is_done_after_duration_of_reference_time():
    m = RunMonitor(10.0)
    assert not m.started and not m.done and m.elapsed == 0.0
    assert not feed(m, 100.0, 109.98)
    assert m.on_reference(110.0)
    assert m.done and m.elapsed == 10.0 and m.references == 1000


def test_status_counts_modes_alarms_and_first_alarm_relative_to_first_reference():
    m = RunMonitor(60.0, namespaces=('ids',))
    m.on_status('ids', 1.0, 0, False)
    m.on_reference(2.0)
    m.on_status('ids', 2.5, 2, False)
    m.on_status('ids', 32.25, 2, True)
    m.on_status('ids', 32.26, 2, True)
    s = m.summary('complete')['ids']['ids']
    assert s == {'status_messages': 4, 'alarm_samples': 2, 'first_alarm': 30.25,
                 'modes': {'waiting': 1, 'armed': 3}}


def test_warnings_point_at_the_missing_piece():
    m = RunMonitor(5.0)
    warnings = m.warnings()
    assert any('/plc/reference' in w for w in warnings)
    assert any('/ids/status' in w for w in warnings)
    assert any('/ids_thesis/status' in w for w in warnings)
    m.on_reference(1.0)
    m.on_status('ids', 1.0, 3, False)
    m.on_status('ids_thesis', 1.0, 2, False)
    assert '/robot/measurement' in m.warnings()[0]
    m.on_measurement([0.0] * 7)
    warnings = m.warnings()
    assert warnings == ['/ids reported FAULT: missing calibration or fingerprint mismatch']


def test_summary_and_progress_report_the_run():
    m = RunMonitor(20.0, namespaces=('ids',))
    feed(m, 0.0, 20.0, step=0.5)
    m.on_status('ids', 3.0, 1, False)
    m.on_measurement([10.0] * 7)
    summary = m.summary('complete')
    assert summary['status'] == 'complete'
    assert summary['elapsed'] == 20.0 and summary['references'] == 41
    assert summary['measurements'] == 1 and summary['torque_saturation'] == 0.0
    assert summary['warnings'] == []
    assert m.progress() == 't=20/20 s, references 41, ids: 1 status, 0 alarm'


def test_torque_saturation_is_counted_after_the_first_reference_and_flagged():
    m = RunMonitor(10.0, namespaces=(), saturation_warning=0.05)
    m.on_measurement([320.0, 0, 0, 0, 0, 0, 0])
    assert m.measurements == 0 and m.torque_saturation is None
    m.on_reference(0.0)
    for k in range(100):
        efforts = [0.0] * 7
        if k < 5:
            efforts[2] = -176.0
        elif k == 5:
            efforts[6] = 39.99
        m.on_measurement(efforts)
    assert m.measurements == 100 and m.saturated == 5
    assert m.warnings() == []
    m.on_measurement([0.0, 0.0, 176.0, 0.0, 0.0, 0.0, 0.0])
    summary = m.summary('complete')
    assert summary['torque_saturation'] == round(6 / 101, 4)
    assert len(summary['warnings']) == 1 and 'not nominal' in summary['warnings'][0]