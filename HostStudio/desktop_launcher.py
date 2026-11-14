"""2416 Studio: offline desktop, no web server and no external browser."""
import argparse
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import sys
from PySide6.QtCore import QLockFile, QStandardPaths
from PySide6.QtWidgets import QApplication, QMessageBox
from desktop.window import Window

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--fullscreen',action='store_true')
    parser.add_argument('--smoke-report',type=Path)
    args=parser.parse_args()
    app=QApplication(sys.argv[:1]); app.setApplicationName('2416 Studio'); app.setOrganizationName('2416 Studio')
    data=Path(os.getenv('LOCALAPPDATA',str(Path.home())))/'2416Studio'; data.mkdir(parents=True,exist_ok=True)
    logging.basicConfig(level=logging.INFO,handlers=[RotatingFileHandler(data/'desktop.log',maxBytes=2000000,backupCount=2,encoding='utf-8')])
    # Mock-only validation has its own lock and never touches a running hardware session.
    lock_name='2416-Studio-smoke.lock' if args.smoke_report else '2416-Studio.lock'
    lock=QLockFile(str(Path(QStandardPaths.writableLocation(QStandardPaths.StandardLocation.TempLocation))/lock_name))
    if not lock.tryLock(0):
        QMessageBox.information(None,'2416 Studio','设备控制台已经运行，请切换到现有窗口。'); return 1
    root=Path(getattr(sys,'_MEIPASS',Path(__file__).resolve().parent))/'web/dist'
    if not (root/'index.html').exists():
        QMessageBox.critical(None,'界面资源缺失','请先在 HostStudio/web 目录运行 npm ci 和 npm run build。'); return 1
    window=Window(root)
    window.showFullScreen() if args.fullscreen else window.show()
    if args.smoke_report:
        from desktop.smoke import Smoke
        window.smoke=Smoke(window,args.smoke_report)
    result=app.exec(); lock.unlock(); return result

if __name__=='__main__': raise SystemExit(main())
