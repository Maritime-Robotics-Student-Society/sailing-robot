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
}
</style>
</head>
<body>
{% for day, batch in days %}
  <div class="day">
    <h2>{{day.strftime('%a %d %B %Y')}}</h2>
    {% for run in batch %}
    <div class="run">
      {{run.rosbag.start.strftime('%H:%M:%S')}} [{{run.rosbag.test_name.strip('_')}}] :
      {% for file in run %}
        <a href="file://{{file.path}}">{{file.file_type}}</a> ·
      {% endfor %}
    </div>
    {% endfor %}
  </div>
{% endfor %}

</body>
</html>
