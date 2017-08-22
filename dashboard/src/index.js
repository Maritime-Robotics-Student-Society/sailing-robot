// id = document.getElementById.bind(document);
// elem = document.createElement.bind(document);
// text = document.createTextNode.bind(document);

// function construct(tag, children) {
//   var res = document.createElement(tag);
//   children.forEach(function(c) {
//     res.appendChild(c);
//   })
//   return res;
// }

function formatValue(msg) {
  if (msg.latitude !== undefined) {
    const latHemi = msg.latitude > 0 ? 'N' : 'S';
    const lonHemi = msg.longitude > 0 ? 'E' : 'W';
    return `${Math.abs(msg.latitude)}° ${latHemi} / ${Math.abs(msg.longitude)}°
       ${lonHemi}}`;
  } else {
    return msg.value;
  }
}

function updateTopicTable(msg) {
  const topicName = msg.topic;
  const topicsList = topicsTable.topics;
  const topic = topicsList.find(t => t.name === topicName);
  if (topic) {
    topic.value = formatValue(msg);
  } else {
    // Add new row to table
    topicsList.push({
      name: topicName,
      value: formatValue(msg)
    });
  }
}

// function rosout_handling(target_id) {
//   var target = id(target_id);
//   var output_table = target.querySelector('#rosout-display');
//   var recent_msgs = [];
//   var keep_n_msgs = 1000;
//   var trim_at_n_msgs = 1500;
//   var show_n_msgs = 20;
//   var current_level = 4;  // 4: warning

//   function rerender() {
//     var new_tbody = construct('tbody', recent_msgs
//           .filter(function(m) {return m.level >= current_level;})
//           .slice(-show_n_msgs)
//           .map(function(m) {return construct('tr', [construct('td', [text(m.msg)])])})
//         );
//     output_table.replaceChild(new_tbody, output_table.firstChild);
//   }

//   function switch_level(level) {
//     current_level = level;
//     rerender();
//   }
//   target.querySelector('#rosout-level-debug').addEventListener('click', function() {switch_level(1);});
//   target.querySelector('#rosout-level-info').addEventListener('click', function() {switch_level(2);});
//   target.querySelector('#rosout-level-warn').addEventListener('click', function() {switch_level(4);});
//   target.querySelector('#rosout-level-error').addEventListener('click', function() {switch_level(8);});
//   target.querySelector('#rosout-level-fatal').addEventListener('click', function() {switch_level(16);});

//   function append_msg(m) {
//     recent_msgs.push(m);
//     if (recent_msgs.length > trim_at_n_msgs) {
//       var n_discard = trim_at_n_msgs - recent_msgs.length;
//       recent_msgs = recent_msgs.slice(n_discard, n_discard + keep_n_msgs);
//     }
//     rerender();
//   }
//   return append_msg;
// }

// function updateCompassHand(name) {
//   return function(msg){
//     id(name+'_hand').style.transform = 'rotate('+msg.value+'deg)';
//   }
// }

// const topicHandlers = {
//   '/heading': [updateCompassHand('heading')],
//   '/goal_heading': [updateCompassHand('goal')],
//   '/dbg_heading_to_waypoint': [updateCompassHand('waypoint')],
//   '/wind_direction_average': [updateCompassHand('wind')],
// };


/////////////////////////////////////////
//          Web Socket Stuff           //
/////////////////////////////////////////
const ws = new WebSocket(`ws://192.168.12.1:8448/updates`);
//var push_rosout = rosout_handling('rosout');
ws.onopen = function (event) {
  connectionOverlays.isDisconnected = false;
  connectionOverlays.isConnecting = false;
};
ws.onmessage = function (event) {
  const jsonMsg = JSON.parse(event.data);
  if (jsonMsg.topic === '/rosout') {
    //push_rosout(json_msg);
  } else {
    updateTopicTable(jsonMsg);
    // (topicHandlers[jsonMsg.topic] || []).forEach(function (h) {
    //   h(jsonMsg);
    // });
  }
};
ws.onclose = function (event) {
  connectionOverlays.isConnecting = false;
  connectionOverlays.isDisconnected = true;
};

const Child = Vue.component('mine-testing', {
  props: {
    wow: {
      type: String,
      default: ''
    }
  },
  template: '<span>{{wow}}</span>',
  data: function () {
    return {
      ech: 'Wowzers'
    }
  }
});

new Vue({
  el: '#test',
  data: {
    ech: 'Yowza'
  },
  components: {
    'mine-testing': Child
  }
});

const connectionOverlays = new Vue({
  el: '#connection-overlays',
  data: {
    isDisconnected: false,
    isConnecting: true
  }
});

const magicCompass = new Vue({
  el: '#magic-compass',
  data: {
    compasses: [{
        name: 'goal_hand',
        style: {},
        bearing: 0
      },
      {
        name: 'heading_hand',
        style: {},
        bearing: 0
      },
      {
        name: 'waypoint_hand',
        style: {},
        bearing: 0
      },
      {
        name: 'wind_hand',
        style: {},
        bearing: 0
      },
    ]
  },
  methods: {
    spin(compass) {
      const deg = compass.style.transform || 10;
      console.log(compass.style);
      console.log(compass.style.transform);
      compass.style.transform = `rotate(${deg}deg)`;
      // let start;
      // function rightRound(timestamp) {
      //   if (start) start = timestamp;
      //   if (timestamp - start > 200) {
      //     start = timestamp;
      //     const deg = n.style.transform || 0;
      //     console.log(deg);
      //   }
      //   requestAnimationFrame(rightRound);
      // }
      // requestAnimationFrame(rightRound);
    }
  }
})

const topicsTable = new Vue({
  el: '#topics-table',
  data: {
    headers: [
      'Topic',
      'Value'
    ],
    topics: range(0, 10).map((_, i) => {
      return Object.assign(Object.create(null), {
        name: `Test${Math.round(10*Math.random())}`,
        value: 30 * Math.random()
      });
    })
  },
  computed: {
    sortedTopics() {
      return this.topics.sort((a, b) => {
        return (a.name.localeCompare(b.name) > 0)? 1 :
        (a.name.localeCompare(b.name === 0))? 0 :
        -1
      });
    }
  }
});
