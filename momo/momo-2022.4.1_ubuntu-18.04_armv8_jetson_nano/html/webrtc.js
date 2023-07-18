const remoteVideo = document.getElementById('remote_video');
const dataTextInput = document.getElementById('data_text');
remoteVideo.controls = true;
let peerConnection = null;
let dataChannel = null;
let candidates = [];
let hasReceivedSdp = false;
// iceServer を定義
const iceServers = [{ 'urls': 'stun:stun.l.google.com:19302' }];
// peer connection の 設定
const peerConnectionConfig = {
  'iceServers': iceServers
};


const isSSL = location.protocol === 'https:';
const wsProtocol = isSSL ? 'wss://' : 'ws://';
const wsUrl = wsProtocol + location.host + '/ws';
const ws = new WebSocket(wsUrl);
ws.onopen = onWsOpen.bind();
ws.onerror = onWsError.bind();
ws.onmessage = onWsMessage.bind();

function onWsError(error){
  console.error('ws onerror() ERROR:', error);
}

function onWsOpen(event) {
  console.log('ws open()');
}
function onWsMessage(event) {
  console.log('ws onmessage() data:', event.data);
  const message = JSON.parse(event.data);
  if (message.type === 'offer') {
    console.log('Received offer ...');
    const offer = new RTCSessionDescription(message);
    console.log('offer: ', offer);
    setOffer(offer);
  }
  else if (message.type === 'answer') {
    console.log('Received answer ...');
    const answer = new RTCSessionDescription(message);
    console.log('answer: ', answer);
    setAnswer(answer);
  }
  else if (message.type === 'candidate') {
    console.log('Received ICE candidate ...');
    const candidate = new RTCIceCandidate(message.ice);
    console.log('candidate: ', candidate);
    if (hasReceivedSdp) {
      addIceCandidate(candidate);
    } else {
      candidates.push(candidate);
    }
  }
  else if (message.type === 'close') {
    console.log('peer connection is closed ...');
  }
}

function connect() {
  console.group();
  if (!peerConnection) {
    console.log('make Offer');
    makeOffer();
  }
  else {
    console.warn('peer connection already exists.');
  }
  console.groupEnd();
}

function disconnect() {
  console.group();
  if (peerConnection) {
    if (peerConnection.iceConnectionState !== 'closed') {
      peerConnection.close();
      peerConnection = null;
      if (ws && ws.readyState === 1) {
        const message = JSON.stringify({ type: 'close' });
        ws.send(message);
      }
      console.log('sending close message');
      cleanupVideoElement(remoteVideo);
      return;
    }
  }
  console.log('peerConnection is closed.');
  console.groupEnd();
}

function drainCandidate() {
  hasReceivedSdp = true;
  candidates.forEach((candidate) => {
    addIceCandidate(candidate);
  });
  candidates = [];
}

function addIceCandidate(candidate) {
  if (peerConnection) {
    peerConnection.addIceCandidate(candidate);
  }
  else {
    console.error('PeerConnection does not exist!');
  }
}

function sendIceCandidate(candidate) {
  console.log('---sending ICE candidate ---');
  const message = JSON.stringify({ type: 'candidate', ice: candidate });
  console.log('sending candidate=' + message);
  ws.send(message);
}

function playVideo(element, stream) {
  element.srcObject = stream;
}

function prepareNewConnection() {
  const peer = new RTCPeerConnection(peerConnectionConfig);
  dataChannel = peer.createDataChannel("serial");
  if ('ontrack' in peer) {
    if (isSafari()) {
      let tracks = [];
      peer.ontrack = (event) => {
        console.log('-- peer.ontrack()');
        tracks.push(event.track)
        // safari で動作させるために、ontrack が発火するたびに MediaStream を作成する
        let mediaStream = new MediaStream(tracks);
        playVideo(remoteVideo, mediaStream);
      };
    }
    else {
      let mediaStream = new MediaStream();
      playVideo(remoteVideo, mediaStream);
      peer.ontrack = (event) => {
        console.log('-- peer.ontrack()');
        mediaStream.addTrack(event.track);
      };
    }
  }
  else {
    peer.onaddstream = (event) => {
      console.log('-- peer.onaddstream()');
      playVideo(remoteVideo, event.stream);
    };
  }

  peer.onicecandidate = (event) => {
    console.log('-- peer.onicecandidate()');
    if (event.candidate) {
      console.log(event.candidate);
      sendIceCandidate(event.candidate);
    } else {
      console.log('empty ice event');
    }
  };

  peer.oniceconnectionstatechange = () => {
    console.log('-- peer.oniceconnectionstatechange()');
    console.log('ICE connection Status has changed to ' + peer.iceConnectionState);
    switch (peer.iceConnectionState) {
      case 'closed':
      case 'failed':
      case 'disconnected':
        break;
    }
  };
  peer.addTransceiver('video', {direction: 'recvonly'});
  peer.addTransceiver('audio', {direction: 'recvonly'});

  dataChannel.onmessage = function (event) {
    console.log("Got Data Channel Message:", new TextDecoder().decode(event.data));
  };
  
  return peer;
}

function browser() {
  const ua = window.navigator.userAgent.toLocaleLowerCase();
  if (ua.indexOf('edge') !== -1) {
    return 'edge';
  }
  else if (ua.indexOf('chrome')  !== -1 && ua.indexOf('edge') === -1) {
    return 'chrome';
  }
  else if (ua.indexOf('safari')  !== -1 && ua.indexOf('chrome') === -1) {
    return 'safari';
  }
  else if (ua.indexOf('opera')   !== -1) {
    return 'opera';
  }
  else if (ua.indexOf('firefox') !== -1) {
    return 'firefox';
  }
  return ;
}

function isSafari() {
  return browser() === 'safari';
}

function sendSdp(sessionDescription) {
  console.log('---sending sdp ---');
  const message = JSON.stringify(sessionDescription);
  console.log('sending SDP=' + message);
  ws.send(message);
}

async function makeOffer() {
  peerConnection = prepareNewConnection();
  try {
    const sessionDescription = await peerConnection.createOffer({
      'offerToReceiveAudio': true,
      'offerToReceiveVideo': true
    })
    console.log('createOffer() success in promise, SDP=', sessionDescription.sdp);
    switch (document.getElementById('codec').value) {
      case 'H264':
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP8');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP9');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'AV1');
        break;
      case 'VP8':
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'H264');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP9');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'AV1');
        break;
      case 'VP9':
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'H264');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP8');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'AV1');
        break;
      case 'AV1':
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'H264');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP8');
        sessionDescription.sdp = removeCodec(sessionDescription.sdp, 'VP9');
        break;
    }
    await peerConnection.setLocalDescription(sessionDescription);
    console.log('setLocalDescription() success in promise');
    sendSdp(peerConnection.localDescription);
  } catch (error) {
    console.error('makeOffer() ERROR:', error);
  }
}

async function makeAnswer() {
  console.log('sending Answer. Creating remote session description...');
  if (!peerConnection) {
    console.error('peerConnection DOES NOT exist!');
    return;
  }
  try {
    const sessionDescription = await peerConnection.createAnswer();
    console.log('createAnswer() success in promise');
    await peerConnection.setLocalDescription(sessionDescription);
    console.log('setLocalDescription() success in promise');
    sendSdp(peerConnection.localDescription);
    drainCandidate();
  } catch (error) {
    console.error('makeAnswer() ERROR:', error);
  }
}

// offer sdp を生成する
function setOffer(sessionDescription) {
  if (peerConnection) {
    console.error('peerConnection already exists!');
  }
  const peerConnection = prepareNewConnection();
  peerConnection.onnegotiationneeded = async function () {
    try{
      await peerConnection.setRemoteDescription(sessionDescription);
      console.log('setRemoteDescription(offer) success in promise');
      makeAnswer();
    }catch(error) {
      console.error('setRemoteDescription(offer) ERROR: ', error);
    }
  }
}

async function setAnswer(sessionDescription) {
  if (!peerConnection) {
    console.error('peerConnection DOES NOT exist!');
    return;
  }
  try {
    await peerConnection.setRemoteDescription(sessionDescription);
    console.log('setRemoteDescription(answer) success in promise');
    drainCandidate();
  } catch(error) {
    console.error('setRemoteDescription(answer) ERROR: ', error);
  }
}

function cleanupVideoElement(element) {
  element.pause();
  element.srcObject = null;
}


/* getOffer() function is currently unused.
function getOffer() {
  initiator = false;
  createPeerConnection();
  sendXHR(
    ".GetOffer",
    JSON.stringify(peer_connection.localDescription),
    function (respnse) {
      peer_connection.setRemoteDescription(
        new RTCSessionDescription(respnse),
        function () {
          peer_connection.createAnswer(
            function (answer) {
              peer_connection.setLocalDescription(answer);
            }, function (e) { });
        }, function (e) {
          console.error(e);
        });
    }, true);
}
*/

// Stack Overflow より引用: https://stackoverflow.com/a/52760103
// https://stackoverflow.com/questions/52738290/how-to-remove-video-codecs-in-webrtc-sdp
function removeCodec(orgsdp, codec) {
  const internalFunc = (sdp) => {
    const codecre = new RegExp('(a=rtpmap:(\\d*) ' + codec + '\/90000\\r\\n)');
    const rtpmaps = sdp.match(codecre);
    if (rtpmaps == null || rtpmaps.length <= 2) {
      return sdp;
    }
    const rtpmap = rtpmaps[2];
    let modsdp = sdp.replace(codecre, "");

    const rtcpre = new RegExp('(a=rtcp-fb:' + rtpmap + '.*\r\n)', 'g');
    modsdp = modsdp.replace(rtcpre, "");

    const fmtpre = new RegExp('(a=fmtp:' + rtpmap + '.*\r\n)', 'g');
    modsdp = modsdp.replace(fmtpre, "");

    const aptpre = new RegExp('(a=fmtp:(\\d*) apt=' + rtpmap + '\\r\\n)');
    const aptmaps = modsdp.match(aptpre);
    let fmtpmap = "";
    if (aptmaps != null && aptmaps.length >= 3) {
      fmtpmap = aptmaps[2];
      modsdp = modsdp.replace(aptpre, "");

      const rtppre = new RegExp('(a=rtpmap:' + fmtpmap + '.*\r\n)', 'g');
      modsdp = modsdp.replace(rtppre, "");
    }

    let videore = /(m=video.*\r\n)/;
    const videolines = modsdp.match(videore);
    if (videolines != null) {
      //If many m=video are found in SDP, this program doesn't work.
      let videoline = videolines[0].substring(0, videolines[0].length - 2);
      const videoelems = videoline.split(" ");
      let modvideoline = videoelems[0];
      videoelems.forEach((videoelem, index) => {
        if (index === 0) return;
        if (videoelem == rtpmap || videoelem == fmtpmap) {
          return;
        }
        modvideoline += " " + videoelem;
      })
      modvideoline += "\r\n";
      modsdp = modsdp.replace(videore, modvideoline);
    }
    return internalFunc(modsdp);
  }
  return internalFunc(orgsdp);
}

function play() {
  remoteVideo.play();
}

function sendDataChannel() {
  let textData = dataTextInput.value;
  if (textData.length == 0) {
    return;
  }
  if (dataChannel == null || dataChannel.readyState != "open") {
    return;
  }
  dataChannel.send(new TextEncoder().encode(textData));
  dataTextInput.value = "";
}

/*
 * Gamepad API Test
 * Written in 2013 by Ted Mielczarek <ted@mielczarek.org>
 *
 * To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this software to the public domain worldwide. This software is distributed without any warranty.
 *
 * You should have received a copy of the CC0 Public Domain Dedication along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
 */

var haveEvents = 'GamepadEvent' in window;
var controllers = {};
var rAF = window.requestAnimationFrame;

function connectHandler(e) {
    addGamepad(e.gamepad);
}
function addGamepad(gamepad) {
    // gamepadのArrayを作成
    controllers[gamepad.index] = gamepad;
    // HTMLへ接続されたGamepad毎の要素を追加（複数のgamepadにも対応）
    var d = document.createElement("div");
    d.setAttribute("id", "controller" + gamepad.index);//idはpadの番号がついた形式
    var t = document.createElement("h2");
    t.appendChild(document.createTextNode("接続Gamepad情報: "));
    d.appendChild(t);
    var info = document.createElement("h1");
    info.appendChild(document.createTextNode(gamepad.id));
    d.appendChild(info);

    //Gamepadコントロール要素（ボタンなど）表示部分
    var b = document.createElement("div");
    b.className = "buttons";
    var t = document.createElement("h2");
    t.appendChild(document.createTextNode("ボタンコントロール情報: "));
    b.appendChild(t);
    for (var i = 0; i < gamepad.buttons.length; i++) {
        var e = document.createElement("span");
        e.className = "button";
        //e.id = "b" + i;
        e.innerHTML = i;
        b.appendChild(e);
    }
    d.appendChild(b);

    //Gamepadコントロール要素（アナログジョイなど）表示部分
    var a = document.createElement("div");
    a.className = "axes";
    var t = document.createElement("h2");
    t.appendChild(document.createTextNode("アナログコントロール情報: "));
    a.appendChild(t);
    for (i = 0; i < gamepad.axes.length; i++) {
        c = document.createElement("h3");
        c.appendChild(document.createTextNode("axis" + i));
        a.appendChild(c);
        e = document.createElement("meter");
        e.className = "axis";
        //e.id = "a" + i;
        e.setAttribute("min", "-1");
        e.setAttribute("max", "1");
        e.setAttribute("value", "0");
        e.innerHTML = i;
        a.appendChild(e);
    }
    d.appendChild(a);
    document.getElementById("start").style.display = "none";
    document.body.appendChild(d);
    rAF(updateStatus);
}

function disconnectHandler(e) {
    removeGamepad(e.gamepad);
}

function removeGamepad(gamepad) {
    var d = document.getElementById("controller" + gamepad.index);
    document.body.removeChild(d);
    delete controllers[gamepad.index];
}

function updateStatus() {
    scanGamepads();
    for (j in controllers) {
        var controller = controllers[j];
        var d = document.getElementById("controller" + j);
        var buttons = d.getElementsByClassName("button");

        //ボタン情報の状態取得
        for (var i = 0; i < controller.buttons.length; i++) {
            var b = buttons[i];
            var val = controller.buttons[i];
            var pressed = val == 1.0;
            if (typeof (val) == "object") {
                pressed = val.pressed;
                val = val.value;
            }
            var pct = Math.round(val * 100) + "%";
            b.style.backgroundSize = pct + " " + pct;
            if (pressed) {
                b.className = "button pressed";
            } else {
                b.className = "button";
            }
        }
        //アナログコントロール情報の状態取得
        var axes = d.getElementsByClassName("axis");
        for (var i = 0; i < controller.axes.length; i++) {
            var a = axes[i];
            a.innerHTML = i + ": " + controller.axes[i].toFixed(4);
            a.setAttribute("value", controller.axes[i]);
        }
        sendGamepadData(controller);
    }
}
   
function sendGamepadData(gamepad) {
      // ゲームパッドのボタンと軸の値を取得し、シリアル通信で送信する処理を実装する
      // ここでは、値をコンソールに出力する例としています
      let needData = gamepad.axes.slice(0,2); 
      //console.log("Button Values:", gamepad.buttons.map(button => button.value));
      
      const x_MAX=-1;      
      const y_MAX=-1;
      

      needData[0]*=x_MAX;//x
      needData[1]*=y_MAX;//y

      let x=Math.round(needData[0],3);
      let y=Math.round(needData[1],3);
      
      console.log("Axis Values:", String(x),String(y));
      dataChannel.send(String(x)+","+String(y)+"\n");
  rAF(updateStatus);
}



function scanGamepads() {
    var gamepads = navigator.getGamepads();
    for (var i = 0; i < gamepads.length; i++) {
        if (gamepads[i]) {
            if (!(gamepads[i].index in controllers)) {
                addGamepad(gamepads[i]);
                console.log("a");
            } else {
                controllers[gamepads[i].index] = gamepads[i];
                //console.log("b");
            }
        }
    }
}

if (haveEvents) {
    window.addEventListener("gamepadconnected", connectHandler);
    window.addEventListener("gamepaddisconnected", disconnectHandler);
} else {
    setInterval(scanGamepads, 500);
}



