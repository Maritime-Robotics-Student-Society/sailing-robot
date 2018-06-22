{# This is the template for an index of recorded data files.
  It is used by index_recorded_data.py #}
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />
<title>Dataset Index</title>
<style>
body {
  font-family: sans-serif;
  background-color: #ddd;
  margin: 0;
}
#container {
  max-width: 1000px;
  margin: 0 auto; /* Centred */
  padding: 10px;
  background-color: #fff;
}
.day-notes-switch {
  margin-bottom: 1em;
}
.run {
  margin-bottom: 0.5em;
}
.bag-line {
  padding-left: 10px;
}
.topics-line {
  padding-left: 10px;
  font-size: 10pt;
  font-family: monospace;
  line-height: 160%;
}
.topic-name {
  padding: 2px;
  background-color: #eee;
}
.meter-bar-outer {
  display: inline-block;
  width: 100px;
  height: 0.8em;
}
.meter-bar-inner {
  background-color: #494;
  height: 100%
}
.map {
  width: 50%;
  height: 300px;
}
.map iframe {
  width: 100%;
  height: 100%;
  border-width: 0;
}
.hidden {
  display: none;
}
</style>
<script>
var find_id = document.getElementById.bind(document);
var create = document.createElement.bind(document);

function toggle_hidden(id) {
  var el = document.getElementById(id);
  el.classList.toggle('hidden');
  return false;
}

function toggle_map(map_id, filename, run_id) {
  var map_el = find_id(map_id);
  if (map_el) {
    map_el.classList.toggle('hidden');
  } else {
    map_el = create('div');
    map_el.id = map_id;
    map_el.classList.add('map');
    var iframe_el = create('iframe');
    map_el.appendChild(iframe_el);
    iframe_el.setAttribute('src', filename);
    //iframe_el.attributes['src'] = filename;
    find_id(run_id).appendChild(map_el);
  }
  return false;
}
</script>
</head>
<body>
<div id="container">
{% for date, day_data in days %}
  <div class="day">
    <h2>{{date.strftime('%a %d %B %Y')}}</h2>

    <!-- First, list all notes from this day (if there are any) -->
    {% set day_notes_id = date.strftime('day-notes-%Y-%m-%d') %}
    {%- if day_data.notes -%}
    <div class="day-notes-switch">
      <a href="#" onclick="toggle_hidden('{{day_notes_id}}'); return false;">
        {{ day_data.notes | length }} notes</a> made on this day.
    </div>
    <div class="day-notes hidden" id="{{day_notes_id}}">
      <ul>
      {% for note in day_data.notes %}
      <li>{{note['timestamp'].strftime('%H:%M:%S')}} – {{note['message']}}</li>
      {% endfor %}
      </ul>
    </div>
    {%- endif -%}

    <!-- Now show the runs themselves -->
    {% for run in day_data.runs %}
    {% set run_id = run.rosbag.start.strftime('run-%Y-%m-%d-T%H-%M-%S') %}
    {% set topics_id = run.rosbag.start.strftime('topics-%Y-%m-%d-T%H-%M-%S') %}
    {% set notes_id = run.rosbag.start.strftime('notes-%Y-%m-%d-T%H-%M-%S') %}
    {% set map_id = run.rosbag.start.strftime('map-%Y-%m-%d-T%H-%M-%S') %}
    <div class="run" id="{{run_id}}">
      <!-- Start time, log name, links to data files -->
      <div class="run-line">{{run.rosbag.start.strftime('%H:%M:%S')}} [{{run.rosbag.test_name.strip('_')}}] :
        <a id="{{run_id}}"> <a href="./index.html#{{run_id}}">link · </a></a>
      {% for file in run %}
        <a href="{{file.filename}}">{{file.file_type}}</a> ·
      {% endfor %}
      </div>

      <!-- Show run length, number of topics and notes -->
      <div class="bag-line">
        <div class="meter-bar-outer"><div class="meter-bar-inner" style="width:{{run.prop_size}}px;"></div></div>
        {{run.rosbag.duration | seconds_to_mins}} minutes, {{run.rosbag.n_messages}} messages
        in <a href="#" onclick="toggle_hidden('{{topics_id}}'); return false;">{{run.rosbag.topic_list|length}} topics</a>
        {%- if run.osm_map -%}
        , <a href="#" onclick="toggle_map('{{map_id}}', '{{run.osm_map}}', '{{run_id}}'); return false;">map</a>
        {%- endif -%}
        {%- if run.notes -%}
        , with <a href="#" onclick="toggle_hidden('{{notes_id}}'); return false;">{{ run.notes|length }} notes</a>
        {%- endif -%}
      </div>

      <!-- List ROS message topics, and any notes made during this run. -->
      <div class="topics-line hidden" id="{{topics_id}}">
        {% for topic in run.rosbag.topic_list | sort %}
        <span class="topic-name">{{topic}}</span> ·
        {%- endfor %}
      </div>
      <div class="notes-line hidden" id="{{notes_id}}">
        <ul>
        {% for note in run.notes %}
        <li>{{note['timestamp'].strftime('%H:%M:%S')}} – {{note['message']}}</li>
        {% endfor %}
        </ul>
      </div>
    </div>

    {% endfor %}

  </div>
{% endfor %}
</div>
</body>
</html>
