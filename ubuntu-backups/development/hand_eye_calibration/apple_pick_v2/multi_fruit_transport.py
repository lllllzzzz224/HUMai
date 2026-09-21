"""Synchronous guarded adapters around asynchronous ROS APIs.

The inherited validators receive already completed futures, so their legacy
spin timeouts cannot restart or extend the shared deadline. A cancelled local
service future would NOT cancel server work; unresolved services latch a fault.
"""
import time
from multi_fruit_runtime import BudgetExpired, HardwareFault


class GuardedTransport:
    def __init__(self, *, pump, guard, log, clock=time.perf_counter):
        self.pump, self.guard, self.log, self.clock = pump, guard, log, clock
        self.budget = None
        self.fault = None
        self.handles = []
        self.pending = []

    def check(self):
        if self.fault:
            raise HardwareFault(self.fault)
        self.guard()
        if self.budget is not None:
            self.budget.remaining()

    def cancel(self):
        for handle in list(self.handles):
            try:
                handle.cancel_goal_async()
            except Exception:
                pass  # Never infer stopped from failure to contact an action server.

    def wait(self, future, label, *, service=False, timeout_s=30.):
        deadline = self.clock()+timeout_s
        with self.log.phase(label):
            try:
                self.check()
                while not future.done():
                    self.check()
                    remaining = deadline-self.clock()
                    if self.budget is not None:
                        remaining = min(remaining,self.budget.remaining())
                    if remaining <= 0:
                        raise BudgetExpired(f'{label}: response deadline')
                    self.pump(min(.02,remaining))
                self.check()
                return future
            except BudgetExpired:
                if future.done():
                    raise
                self.cancel()
                if not service and self.handles:
                    # A short cancellation drain is explicitly outside the planning budget.
                    end = self.clock()+.5
                    while not future.done() and self.clock()<end:
                        self.pump(.02)
                    if future.done():
                        raise
                self.pending.append(future)
                self.fault = f'{label}: remote completion unconfirmed; no subsequent target allowed'
                raise HardwareFault(self.fault)
            except BaseException:
                if not future.done():
                    self.pending.append(future)
                    self.fault = f'{label}: interrupted with remote completion unconfirmed'
                self.cancel()
                raise

    def service(self, client, label):
        transport = self
        class Service:
            def __getattr__(self,name): return getattr(client,name)
            def call_async(self, request):
                transport.check()
                if transport.budget is not None and hasattr(request,'ik_request'):
                    timeout = request.ik_request.timeout
                    seconds = min(timeout.sec+timeout.nanosec*1e-9,transport.budget.remaining())
                    timeout.sec = int(seconds)
                    timeout.nanosec = int((seconds-int(seconds))*1e9)
                future = client.call_async(request)
                return transport.wait(future,label,service=True)
        return Service()

    def action(self, client, label, *, timeout_s=30.):
        transport = self
        class Handle:
            def __init__(self,handle): self.handle=handle; self.accepted=handle.accepted
            def get_result_async(self):
                future = self.handle.get_result_async()
                try:
                    return transport.wait(future,label,timeout_s=timeout_s)
                finally:
                    if future.done() and self.handle in transport.handles:
                        transport.handles.remove(self.handle)
        class Action:
            def __getattr__(self,name): return getattr(client,name)
            def send_goal_async(self,goal):
                from rclpy.task import Future
                transport.check()
                if transport.budget is not None and hasattr(goal,'request'):
                    goal.request.allowed_planning_time = min(goal.request.allowed_planning_time,
                                                             transport.budget.remaining())
                future = client.send_goal_async(goal)
                def accepted(done):
                    try:
                        handle=done.result()
                    except BaseException:
                        transport.fault = f'{label}: goal acceptance failed; completion unconfirmed'
                        return
                    if handle is not None and handle.accepted:
                        transport.handles.append(handle)
                        if transport.fault:
                            handle.cancel_goal_async()
                future.add_done_callback(accepted)
                transport.wait(future,label+'_accept',timeout_s=timeout_s)
                handle = future.result()
                completed = Future()
                completed.set_result(None if handle is None else Handle(handle))
                return completed
        return Action()
