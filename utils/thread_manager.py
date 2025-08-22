"""스레드 관리 유틸리티"""

import threading

class ThreadManager:
    """스레드 생명주기 관리"""
    
    def __init__(self):
        self.thread = None
        self.stop_event = threading.Event()
        
    def start(self, target, daemon=True):
        """스레드 시작"""
        if self.thread is None or not self.thread.is_alive():
            self.stop_event.clear()
            self.thread = threading.Thread(target=target, daemon=daemon)
            self.thread.start()
            
    def stop(self, timeout=1.0):
        """스레드 정지"""
        if self.thread and self.thread.is_alive():
            self.stop_event.set()
            self.thread.join(timeout=timeout)
            
    def should_stop(self):
        """정지 신호 확인"""
        return self.stop_event.is_set()