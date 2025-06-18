// ROSへ接続
const ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'  // rosbridgeが動作するマシン・ポートに合わせて修正
  });
  
  ros.on('connection', () => {
    console.log('Connected to rosbridge websocket server.');
  });
  
  ros.on('error', (error) => {
    console.error('Error connecting to rosbridge:', error);
  });
  
  ros.on('close', () => {
    console.log('Connection to rosbridge closed.');
  });
  
  // 4つのトピック購読設定
  // 各トピックの型はstd_msgs/Int32を想定。もし別の型ならmessageTypeを変更
  const topic1 = new ROSLIB.Topic({
    ros : ros,
    name : '/gpt_output1',
    messageType : 'std_msgs/Int32'
  });
  
  const topic2 = new ROSLIB.Topic({
    ros : ros,
    name : '/gpt_output2',
    messageType : 'std_msgs/Int32'
  });
  
  const topic3 = new ROSLIB.Topic({
    ros : ros,
    name : '/gpt_output3',
    messageType : 'std_msgs/Int32'
  });
  
  const topic4 = new ROSLIB.Topic({
    ros : ros,
    name : '/gpt_output4',
    messageType : 'std_msgs/Int32'
  });
  
  // データ受信時のコールバック
  topic1.subscribe((message) => {
    document.getElementById('data1').textContent = message.data;
  });
  
  topic2.subscribe((message) => {
    document.getElementById('data2').textContent = message.data;
  });
  
  topic3.subscribe((message) => {
    document.getElementById('data3').textContent = message.data;
  });
  
  topic4.subscribe((message) => {
    document.getElementById('data4').textContent = message.data;
  });
  