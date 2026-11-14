import json
import logging
from pathlib import Path
from PySide6.QtCore import QObject, Slot, Signal, QUrl, QEvent, Qt, QTimer
from PySide6.QtWidgets import QMainWindow, QApplication
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWebEngineCore import QWebEnginePage, QWebEngineSettings, QWebEngineUrlRequestInterceptor
from PySide6.QtWebChannel import QWebChannel
from backend.controller import Controller

class LocalResources(QWebEngineUrlRequestInterceptor):
    def __init__(self,root,parent=None): super().__init__(parent); self.root=root.resolve()
    def interceptRequest(self,info):
        url=info.requestUrl()
        allowed=url.scheme() in ('qrc','data','blob') or (url.isLocalFile() and Path(url.toLocalFile()).resolve().is_relative_to(self.root))
        if not allowed: info.block(True)

class Page(QWebEnginePage):
    def __init__(self,root,parent): super().__init__(parent); self.root=root.resolve()
    def acceptNavigationRequest(self,url,kind,main):
        return url.isLocalFile() and Path(url.toLocalFile()).resolve()==self.root/'index.html'
    def javaScriptConsoleMessage(self,level,message,line,source): logging.info('Web %s:%s %s',source,line,message)

class Bridge(QObject):
    snapshot=Signal(str)
    reply=Signal(str)
    def __init__(self,worker,parent):
        super().__init__(parent); self.worker=worker; self.active=True
        worker.snapshot.connect(self.snapshot); worker.reply.connect(self.reply)
    @Slot(str)
    def request(self,raw):
        try:
            req=json.loads(raw)
            if not isinstance(req,dict) or not isinstance(req.get('action'),str): raise ValueError('请求格式错误')
            if not self.active and req['action'] in ('jog','renew'): raise ValueError('窗口未激活，点动已取消')
            self.worker.submit(req)
        except Exception as exc: self.reply.emit(json.dumps(dict(id='',ok=False,error=str(exc))))

class Window(QMainWindow):
    def __init__(self,root):
        super().__init__()
        self.setWindowTitle('2416 Studio · 设备控制台')
        self.resize(1440,960); self.setMinimumSize(1000,700)
        self.setStyleSheet('QMainWindow { background: #F6F8FA; }')
        self.worker=Controller(); self.bridge=Bridge(self.worker,self)
        self.view=QWebEngineView(self); self.page=Page(root,self.view); self.view.setPage(self.page)
        self.channel=QWebChannel(self.page); self.channel.registerObject('hmi',self.bridge); self.page.setWebChannel(self.channel)
        self.interceptor=LocalResources(root,self.page); self.page.profile().setUrlRequestInterceptor(self.interceptor)
        self.page.profile().downloadRequested.connect(lambda d:d.cancel())
        self.page.settings().setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls,False)
        self.page.settings().setAttribute(QWebEngineSettings.WebAttribute.JavascriptCanOpenWindows,False)
        self.view.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)
        self.setCentralWidget(self.view); self.closing=False; self.loaded=False
        self.worker.finished.connect(self.closed)
        self.view.loadStarted.connect(self.loading)
        self.view.loadFinished.connect(self.load_finished)
        self.view.renderProcessTerminated.connect(lambda *args:self.loading())
        QApplication.instance().applicationStateChanged.connect(lambda *args:self.activation())
        self.worker.start(); self.view.load(QUrl.fromLocalFile(str(root.resolve()/'index.html')))

    def loading(self):
        self.loaded=False; self.bridge.active=False; self.worker.suspend()
    def load_finished(self,ok):
        self.loaded=ok; self.activation()
        if not ok: self.statusBar().showMessage('页面加载失败，请检查 web/dist 资源')
    def activation(self):
        active=self.loaded and self.isActiveWindow() and not self.isMinimized() and not self.closing
        if not active and self.bridge.active:
            self.worker.suspend()
        self.bridge.active=active
    def changeEvent(self,event):
        super().changeEvent(event)
        if hasattr(self,'bridge') and event.type() in (QEvent.Type.ActivationChange,QEvent.Type.WindowStateChange): self.activation()
    def closeEvent(self,event):
        if self.worker.isRunning():
            event.ignore()
            if not self.closing:
                self.closing=True; self.bridge.active=False; self.view.setEnabled(False)
                self.statusBar().showMessage('正在停止点动并关闭通信…'); self.worker.close()
        else: event.accept()
    def closed(self):
        if self.closing: QTimer.singleShot(0,self.close)
