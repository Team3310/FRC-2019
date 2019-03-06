<%@taglib uri='http://java.sun.com/jsp/jstl/core' prefix='c' %>
<!DOCTYPE html>
<html>
<head>
    <title>Hawk Path</title>

    <script src='https://ajax.googleapis.com/ajax/libs/jquery/3.1.1/jquery.min.js'></script>
    <script src='https://ajax.googleapis.com/ajax/libs/jqueryui/1.12.1/jquery-ui.min.js'></script>
    <script type='text/javascript' src='<c:url value='/resources/js/script.js' />'></script>

    <link rel='shortcut icon' href='/resources/img/favicon.ico'/>
    <link href='https://fonts.googleapis.com/css?family=Roboto' rel='stylesheet'>
    <link href='https://fonts.googleapis.com/css?family=Source+Code+Pro' rel='stylesheet'>
    <link href='https://fonts.googleapis.com/css?family=Source+Code+Pro' rel='stylesheet'>
    <link href='<c:url value='/resources/css/style.css' />' rel='stylesheet'>
</head>
<body onload='init()'>
<input id='title' placeholder='Title'>
<div id='canvases'>
    <canvas id='background'></canvas>
    <canvas id='field'></canvas>
</div>
<div class='buttonContainer'>
    <select onchange='importTrajectory()' id="trajectory-list">
        <option selected value="base">Select Trajectory</option>
	</select>
    <button onclick='addPoint()'>Add Point</button>
    <button onclick='update()'>Update</button>
    <button onclick='draw(3)'>Animate</button>
    <button onclick='flipField()'>Flip Field</button>
    <span class='checkbox'>Is reversed: <input type='checkbox' id='isReversed'></span>
</div>
<table>
    <thead>
    <th></th>
    <th>X</th>
    <th>Y</th>
    <th>Heading</th>
    <th>Comments</th>
    <th>Enabled</th>
    <th>Delete</th>
    </thead>
    <tbody>
    <tr>
    </tr>
    </tbody>
</table>

<input type='file' id='upl' style='display:none;'>
</body>
</html>

<script>
    $('table tbody').sortable({
        helper: fixWidthHelper,
        deactivate: update
    }).disableSelection();

    function fixWidthHelper(e, ui) {
        ui.children().each(function () {
            $(this).width($(this).width());
        });
        return ui;
    }
</script>