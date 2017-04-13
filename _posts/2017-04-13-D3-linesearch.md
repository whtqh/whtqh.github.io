---
title: Wolfe Line Search Example
date: 2017-04-13 14:20:13 +0800
layout: post
permalink: /blog/2014/06/15/D3Demo_Linesearch.html
categories:
  - javascript
  - D3
tags:
  - 前端
---
这是一个关于D3 API 的测试网页


<p> This graph attempts to show how a Wolfe Line Search works.
The goal here is to move downwards along the gradient so that
the loss is reduced sufficiently (controlled by the c1 parameter)
and also that the slope of the loss is decreased sufficiently (controlled by the
c2 parameter). Making sure the slope decreases sufficiently ensures that
we don't take too many short steps. Note that c1 should always be less than c2 or this won't work
appropiately</p>

<p>
The goal here isn't to exactly find the best point along the line, but to
cheaply find a good enough point. The black dots represent points that were calculated as part of doing the line
search. Minimizing the total number of samples taken while still converging
quickly is the goal here:</p>

<div id ="linesearch" >
<div style="text-align:center"><div style="display:inline-block;">
<h4>
    <div class="btn-group">
        <button type="button" class="btn btn-default dropdown-toggle" data-toggle="dropdown">
          <span class="function_sig">
                \(f(x, y) \)
            </span>
            <span class="caret"></span>
        </button>
        <ul class="dropdown-menu" role="menu">
            <li><a class="function_himmelblau">
                \((x^2 + y - 11)^2 + (x+y^2 -7)^2\)
            </a></li>
            <li><a class="function_flower">
                \(x^2 + y^2 + x \sin \left( y \right) + y \sin \left( x \right) \)
            </a></li>
            <li><a class="function_banana">
                \((1-x)^2 + 100 (y - x^2) ^2\)
            </a></li>
            <li><a class="function_matyas">
                \(.26 (x^2 + y^2)  + .48 x y \)
            </a></li>
        </ul>
    </div>
    <span>\(=\)</span>
    <span class="function_label">
        \((1-x)^2 + 100 (y - x^2) ^2\)
    </span>
</h4>
</div></div>

<div id="vis"></div>
<div class="row">
<form class="form-inline" role="form">
    <div class="form-group col-xs-6 col-md-6">
        <div style="text-align:center"><div style="display:inline-block;">
            <label class="r-only" for="c1">C1
                <span id="c1value">= 0.00001</span>
            </label>
        </div></div>
        <div id="c1"></div>
    </div>
    <div class="form-group col-xs-6 col-md-6">
        <div style="text-align:center"><div style="display:inline-block;">
            <label class="r-only" for="c2">C2
                <span id="c2value">= 0.1</span>
            </label>
        </div></div>
        <div id="c2"></div>
    </div>
</form>
</div>
<div style="text-align:center"><div style="display:inline-block;">
<div class="row">
    <div class ="iterations"></div>
</div>
</div>
</div>



<script src="/js/jquery.min.js"></script>
<script src="/js/bootstrap.js"></script>
<script src="/js/d3.min.js"></script>
<script src="/js/fmin.js"></script>
<script src="/js/fmin_vis.js"></script>

<script>
var line_search_plot = new fmin_vis.LineSearchContour(d3.select("#linesearch"));
</script>

<script type="text/x-mathjax-config">
  MathJax.Hub.Config({
    showMathMenu: false,
    extensions: ["tex2jax.js"],
    jax: ["input/TeX", "output/HTML-CSS"],
    tex2jax: {
      inlineMath: [ ['$','$'], ["\\(","\\)"] ],
      displayMath: [ ['$$','$$'], ["\\[","\\]"] ],
      processEscapes: true
    },
    "HTML-CSS": { availableFonts: ["TeX"] }
  });
</script>
<script type="text/javascript" src="https://cdn.mathjax.org/mathjax/latest/MathJax.js"></script>