import importlib.util
import unittest
from test_multi_fruit_state import snapshot, detection

class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('multi_fruit_runtime'), 'bounded runtime is missing')
        import multi_fruit_runtime as runtime
        self.runtime = runtime

    def test_budget_is_one_absolute_deadline(self):
        now=[10.]
        budget=self.runtime.Budget(2.,clock=lambda:now[0])
        now[0]=11.5
        self.assertAlmostEqual(budget.remaining(),.5)
        now[0]=12.
        with self.assertRaises(self.runtime.BudgetExpired): budget.remaining()

    def test_interlock_rejects_missing_stale_replayed_false_and_wrong_source(self):
        gate=self.runtime.Interlock('safety-plc',max_age_s=.5)
        self.assertFalse(gate.safe(10.,20.))
        good=dict(version=1,source='safety-plc',stamp_s=10.,sequence=1,healthy=True,clear=True)
        gate.update(good,10.,20.)
        self.assertTrue(gate.safe(10.1,20.1))
        self.assertFalse(gate.safe(10.6,20.6))
        gate.update(good,10.7,20.7)
        self.assertFalse(gate.safe(10.7,20.7))
        gate.update(dict(good,stamp_s=11.,sequence=2,clear=False),11.,21.)
        self.assertFalse(gate.safe(11.,21.))
        gate.update(dict(good,source='fake',stamp_s=12.,sequence=3),12.,22.)
        self.assertFalse(gate.safe(12.,22.))

    def test_phase_log_records_nested_inclusive_monotonic_duration_and_failure(self):
        now=[1.]
        log=self.runtime.PhaseLog(clock=lambda:now[0])
        with self.assertRaises(ValueError):
            with log.phase('preflight',target_id='fruit-1'):
                now[0]=1.001
                with log.phase('ik'):
                    now[0]=1.004
                    raise ValueError('bad')
        self.assertEqual(log.spans[0]['status'],'ERROR')
        self.assertAlmostEqual(log.spans[0]['duration_s'],.004)
        self.assertEqual(log.spans[1]['parent_id'],log.spans[0]['id'])

    def test_newer_unsafe_interlock_consumes_watermark_and_blocks_delayed_clear(self):
        for unsafe_field in ('clear','healthy'):
            with self.subTest(unsafe_field=unsafe_field):
                gate=self.runtime.Interlock('safety-plc',max_age_s=.5)
                good=dict(version=1,source='safety-plc',stamp_s=10.,sequence=1,healthy=True,clear=True)
                gate.update(good,10.,20.)
                gate.update(dict(good,stamp_s=10.2,sequence=3,**{unsafe_field:False}),10.2,20.2)
                gate.update(dict(good,stamp_s=10.1,sequence=2),10.3,20.3)
                self.assertFalse(gate.safe(10.3,20.3))
                gate.update(dict(good,stamp_s=10.4,sequence=4),10.4,20.4)
                self.assertTrue(gate.safe(10.4,20.4))

    def test_wait_snapshot_reports_ongoing_interval_without_duplicating_closed_span(self):
        now=[1.]
        log=self.runtime.PhaseLog(clock=lambda:now[0])
        timer=self.runtime.StateWaitTimer(log)
        timer.set('ALL_ATTEMPTED')
        now[0]=4.
        snapshot=timer.snapshot()
        self.assertEqual(snapshot['elapsed_s'],3.)
        self.assertTrue(snapshot['in_progress'])
        self.assertEqual(len(log.spans),0)
        now[0]=6.
        self.assertEqual(timer.snapshot()['elapsed_s'],5.)
        timer.close()
        self.assertIsNone(timer.snapshot())
        self.assertEqual(len(log.spans),1)
        self.assertEqual(log.spans[0]['duration_s'],5.)

if __name__=='__main__': unittest.main()

class TransportTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('multi_fruit_transport'),'guarded transport missing')
        from multi_fruit_transport import GuardedTransport
        from multi_fruit_runtime import PhaseLog, Budget
        self.clock=[10.]
        self.guard_ok=True
        self.transport=GuardedTransport(pump=self.pump,guard=self.guard,log=PhaseLog(),clock=lambda:self.clock[0])
        self.transport.budget=Budget(.1,clock=lambda:self.clock[0])
    def pump(self,seconds): self.clock[0]+=seconds
    def guard(self):
        from multi_fruit_runtime import HardwareFault
        if not self.guard_ok: raise HardwareFault('interlock false')
    def test_unsettled_service_timeout_latches_instead_of_next_request(self):
        from rclpy.task import Future
        from multi_fruit_runtime import HardwareFault
        future=Future()
        with self.assertRaises(HardwareFault): self.transport.wait(future,'ik',service=True)
        self.assertTrue(self.transport.fault)
        self.assertFalse(future.cancelled())
    def test_accepted_action_cancelled_on_interlock_loss(self):
        from rclpy.task import Future
        from multi_fruit_runtime import HardwareFault
        class Handle:
            accepted=True
            def __init__(self): self.cancelled=False
            def cancel_goal_async(self): self.cancelled=True; return Future()
        handle=Handle(); self.transport.handles.append(handle)
        self.guard_ok=False
        with self.assertRaises(HardwareFault): self.transport.wait(Future(),'execution')
        self.assertTrue(handle.cancelled)
    def test_completed_future_cannot_bypass_expired_shared_budget(self):
        from rclpy.task import Future
        from multi_fruit_runtime import BudgetExpired
        future=Future(); future.set_result('ready')
        self.clock[0]=11.
        with self.assertRaises(BudgetExpired): self.transport.wait(future,'ik',service=True)

    def test_pending_acceptance_latches_on_interrupt_and_cancels_late_goal(self):
        from rclpy.task import Future
        future=Future()
        class Client:
            def send_goal_async(self,goal): return future
        class Handle:
            accepted=True
            def __init__(self): self.cancelled=False
            def cancel_goal_async(self): self.cancelled=True; return Future()
        def interrupt(seconds): raise KeyboardInterrupt()
        self.transport.pump=interrupt
        with self.assertRaises(KeyboardInterrupt):
            self.transport.action(Client(),'motion').send_goal_async(object())
        self.assertIsNotNone(self.transport.fault)
        handle=Handle(); future.set_result(handle)
        self.assertTrue(handle.cancelled)
