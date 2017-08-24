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

// function rosoutHandling(targetId) {
//   var target = id(targetId);
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


/////////////////////////////////////////
//          Web Socket Stuff           //
/////////////////////////////////////////
// const ws = new WebSocket(`ws://192.168.12.1:8448/updates`);
// const push_rosout = rosout_handling('rosout');
// ws.onopen = function (ws, event) {
//   connectionOverlays.isDisconnected = false;
//   connectionOverlays.isConnecting = false;
// };
// ws.onmessage = function (ws, event) {
//   const jsonMsg = JSON.parse(event.data);
//   if (jsonMsg.topic === '/rosout') {
//     //push_rosout(jsonMsg);
//   } else {
//     updateTopicTable(jsonMsg);
//     const updateCompassHand = topicHandlers[jsonMsg.topic];
//     if (updateCompassHand) {
//       updateCompassHand(jsonMsg.value);
//     }
//   }
// };
// ws.onerror = function (ws, event) {

// }
// ws.onclose = function (ws, event) {
//   connectionOverlays.isConnecting = false;
//   connectionOverlays.isDisconnected = true;
// };

const connectionOverlays = new Vue({
  el: '#connection-overlays',
  data: {
    isDisconnected: false,
    isConnecting: true
  }
});

const CompassTable = Vue.component('compass-table', {
  props: {
    compasses: {
      type: Array,
      default: []
    }
  },
  template: `<table class="table table-striped table-bordered">
              <thead class="thead-inverse">
                <th>Hand</th>
                <th>Bearing</th>
              </thead>
              <tbody>
                <tr v-for="c of compasses">
                  <th scope="row">{{c.name.replace(/_.+/, '')}}</th>
                  <td>{{c.bearing}}°</td>
                </tr>
              </tbody>
            </table>`
});

const CompassHand = Vue.component('compass-hand', {
  props: {
    compass: {
      type: Object,
      default: Object.create(null)
    }
  },
  template: `<img :src="source"
                  :style="spinEm"
                  :alt="altName"
                  class="compass-hand">`,
  computed: {
    source() {
      return `static/${this.compass.name}.svg`;
    },
    altName() {
      return `Why isn't ${this.compass.name} loaded`;
    },
    spinEm() {
      return Object.assign(Object.create(null), {
        transform: `rotate(${this.compass.bearing}deg)`
      });
    }
  }
});

const magicCompass = new Vue({
  el: '#magic-compass',
  data: {
    compasses: [{
        name: 'goal_hand',
        bearing: 0
      },
      {
        name: 'heading_hand',
        bearing: 0
      },
      {
        name: 'waypoint_hand',
        bearing: 0
      },
      {
        name: 'wind_hand',
        bearing: 0
      },
    ]
  },
  components: {
    'compass-table': CompassTable,
    'compass-hand': CompassHand
  },
  methods: {
    change(name, bearing) {
      const c = this.compasses.find(c => c.name === name);
      c.bearing = bearing % 360;
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
    topics: fp.range(0, 10).map((_, i) => {
      return Object.assign(Object.create(null), {
        name: `Test${Math.round(10*Math.random())}`,
        value: 30 * Math.random()
      });
    })
  },
  computed: {
    sortedTopics() {
      return this.topics.sort((a, b) => {
        return (a.name.localeCompare(b.name) > 0) ? 1 :
          (a.name.localeCompare(b.name === 0)) ? 0 :
          -1
      });
    }
  }
});

const RosoutTable = Vue.component('rosout-table', {
  props: ['displayLevel'],
  template: `<table>
    <tr>
      <td>{{displayLevel}}</td>
    </tr>
  </table>`,
  data: {

  }
});

const rosout = new Vue({
  el: '#rosout',
  data: {
    levels: [
      'Debug',
      'Info',
      'Warn',
      'Error',
      'Fatal'
    ],
    activeLevel: 'Debug'
  },
  methods: {
    levelClass(currentLevel) {
      const r = [];
      // r.push(`rosout-${currentLevel.toLowerCase()}`);
      r.push((this.activeLevel === currentLevel) ? 'active' : '');
      return r.join(' ');
    },
    switchLevel(level) {
      this.activeLevel = level;
    }
  },
  components: {
    'rosout-table': RosoutTable
  }
});

const topicHandlers = {
  '/heading': fp.curry(magicCompass.change, 'heading'),
  '/goal_heading': fp.curry(magicCompass.change, 'goal'),
  '/dbg_heading_to_waypoint': fp.curry(magicCompass.change, 'waypoint'),
  '/wind_direction_average': fp.curry(magicCompass.change, 'wind')
};