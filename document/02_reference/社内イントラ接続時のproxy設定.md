# 社内イントラ接続時のproxy設定

sourcetreeからgithubに接続できなかったが、以下のように解決した。





以下のようにコマンドプロンプトからproxy設定すると接続に成功

参考：<https://qiita.com/HibiU1/items/76ac6edb43cffd9da081>



```
> git config --global http.proxy http://proxy.co.jp:8080
> git config --global https.proxy http://proxy.co.jp:8080
> git config --global url."https://".insteadOf git://
```



proxy設定はpowershellを起動して以下のように確認

```
PS C:\Users\4035207> netsh
netsh>winhttp
netsh winhttp>show proxy

現在の WinHTTP プロキシ設定:

    プロキシ サーバー:  webproxy.mew.co.jp:8080
    バイパス一覧     :  jp0490sw*;10.*.*.*;172.16.*.*;172.17.*.*;172.18.*.*;172.19.*.*;172.2*.*.*;172.30.*.*;172.31.*.*;192.168.*.*;133.254.*.*;133.182.*.*;update.microsoft.com;*.update.microsoft.com;download.windowsupdate.com;*.download.windowsupdate.com;download.microsoft.com;*.download.microsoft.com;windowsupdate.com;*.windowsupdate.com;*.gds.panasonic.com;*.jp.panasonic.com;*.eds.panasonic.com;*.mei.co.jp;*.mew.co.jp;*.intra.panasonic.cn

netsh winhttp>
```



