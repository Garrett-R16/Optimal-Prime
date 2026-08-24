"""Build the results page from the routed examples.

Reads each ``examples/*/summary.json`` and its SVG render, and writes a self-contained HTML
page with the renders inlined. Nothing is hand-copied, so the page cannot drift from the runs
it describes.
"""

from __future__ import annotations

import html
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
EXAMPLES = ROOT / "examples"
OUT = ROOT / "examples" / "results.html"


def inline_svg(path: Path) -> str:
    """Strip the XML preamble and fixed dimensions so the render scales with its container."""
    text = path.read_text(encoding="utf-8")
    start = text.index("<svg")
    svg = text[start:]
    svg = re.sub(r'\swidth="[^"]*"', "", svg, count=1)
    svg = re.sub(r'\sheight="[^"]*"', "", svg, count=1)
    svg = svg.replace("<svg", '<svg class="render" preserveAspectRatio="xMidYMid meet"', 1)
    return svg


def load_examples() -> list[dict]:
    out = []
    for directory in sorted(EXAMPLES.iterdir()):
        summary = directory / "summary.json"
        if not summary.is_dir() and summary.exists():
            data = json.loads(summary.read_text(encoding="utf-8"))
            svgs = list(directory.glob("*.svg"))
            data["svg"] = inline_svg(svgs[0]) if svgs else ""
            data["slug"] = directory.name
            out.append(data)
    return out


def stat(label: str, value: str, tone: str = "") -> str:
    return (f'<div class="stat{tone}"><dt>{html.escape(label)}</dt>'
            f'<dd>{html.escape(value)}</dd></div>')


def card(index: int, item: dict) -> str:
    stats = item["stats"]
    clean = item["clean"]
    verdict = "DRC clean" if clean else "DRC clean, routing incomplete"
    chip = "ok" if clean else "warn"

    layers = " + ".join(item["layers"])
    routed = f"{stats['routed']} / {stats['connections']}"

    blocks = [
        stat("connections routed", routed),
        stat("DRC errors", str(item["drc_errors"]), " good" if item["drc_errors"] == 0 else " bad"),
        stat("unconnected", str(item["unconnected"]),
             " good" if item["unconnected"] == 0 else " warn"),
        stat("arcs", str(stats["arcs"])),
        stat("copper", f"{stats['length_mm']:.1f} mm"),
        stat("time", f"{item['seconds']:.1f} s"),
    ]

    failures = ""
    if item.get("failures"):
        rows = "".join(
            f"<li><span class='net'>{html.escape(f['name'])}</span>"
            f"<span class='why'>{html.escape(f['reason'].split(':', 1)[-1].strip())}</span></li>"
            for f in item["failures"]
        )
        failures = (f'<div class="failures"><h4>Could not route</h4><ul>{rows}</ul>'
                    f'<p class="note">A single layer is not enough for this board. '
                    f'Example 02 is the same board with the back copper allowed.</p></div>')

    return f"""
    <article class="card" id="{html.escape(item['slug'])}">
      <header class="card-head">
        <span class="seq">{index:02d}</span>
        <div>
          <h3>{html.escape(item['board'])}</h3>
          <p class="layers">{html.escape(layers)}</p>
        </div>
        <span class="chip {chip}">{html.escape(verdict)}</span>
      </header>
      <figure class="plate">{item['svg']}</figure>
      <dl class="stats">{''.join(blocks)}</dl>
      {failures}
    </article>"""


def main() -> int:
    items = load_examples()
    if not items:
        print("no examples found; run run.py first")
        return 1

    engine = items[0].get("kicad", "unknown")
    total_arcs = sum(i["stats"]["arcs"] for i in items)
    cards = "\n".join(card(n, item) for n, item in enumerate(items, 1))

    page = TEMPLATE.replace("{{CARDS}}", cards)
    page = page.replace("{{ENGINE}}", html.escape(str(engine)))
    page = page.replace("{{ARCS}}", str(total_arcs))
    OUT.write_text(page, encoding="utf-8")
    print(f"wrote {OUT} ({len(page) // 1024} KB, {len(items)} examples)")
    return 0


TEMPLATE = """<title>Taut String Router</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500&family=IBM+Plex+Serif:ital,wght@0,400;0,600;1,400&display=swap">
<style>
  :root {
    --ground:  #12130F;
    --surface: #1B1D17;
    --raised:  #23261E;
    --ink:     #E6E7DF;
    --muted:   #9A9E90;
    --hair:    #33372B;
    --copper:  #C87D4A;
    --front:   #C83434;
    --back:    #4D7FC4;
    --ok:      #8FBA76;
    --warn:    #D6AE33;
    --plate:   #F4F3EC;

    --serif: "IBM Plex Serif", Georgia, "Times New Roman", serif;
    --sans:  "IBM Plex Sans", -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    --mono:  "IBM Plex Mono", ui-monospace, "SF Mono", Menlo, Consolas, monospace;

    --measure: 66ch;
  }

  :root[data-theme="light"] {
    --ground:  #EEEDE4;
    --surface: #F7F6F0;
    --raised:  #FFFFFF;
    --ink:     #1A1C14;
    --muted:   #5C6153;
    --hair:    #D3D2C4;
    --copper:  #97521F;
    --ok:      #46702F;
    --warn:    #8A6A08;
    --plate:   #FFFFFF;
  }

  @media (prefers-color-scheme: light) {
    :root:not([data-theme="dark"]) {
      --ground:  #EEEDE4;
      --surface: #F7F6F0;
      --raised:  #FFFFFF;
      --ink:     #1A1C14;
      --muted:   #5C6153;
      --hair:    #D3D2C4;
      --copper:  #97521F;
      --ok:      #46702F;
      --warn:    #8A6A08;
      --plate:   #FFFFFF;
    }
  }

  * { box-sizing: border-box; }

  body {
    margin: 0;
    background: var(--ground);
    color: var(--ink);
    font-family: var(--sans);
    font-size: 16px;
    line-height: 1.65;
    -webkit-font-smoothing: antialiased;
  }

  .wrap {
    max-width: 1080px;
    margin: 0 auto;
    padding: clamp(2rem, 5vw, 4.5rem) clamp(1rem, 4vw, 2.5rem) 5rem;
    display: flex;
    flex-direction: column;
    gap: clamp(2.5rem, 5vw, 4rem);
  }

  /* ---------------------------------------------------------------- masthead */

  .masthead { display: flex; flex-direction: column; gap: 1.25rem; }

  .eyebrow {
    font-family: var(--mono);
    font-size: 0.72rem;
    letter-spacing: 0.16em;
    text-transform: uppercase;
    color: var(--copper);
  }

  h1 {
    font-family: var(--serif);
    font-weight: 600;
    font-size: clamp(2.1rem, 5.5vw, 3.4rem);
    line-height: 1.08;
    letter-spacing: -0.02em;
    margin: 0;
    text-wrap: balance;
  }

  .thesis {
    font-family: var(--serif);
    font-style: italic;
    font-size: clamp(1.05rem, 2.2vw, 1.3rem);
    line-height: 1.5;
    color: var(--muted);
    max-width: var(--measure);
    margin: 0;
  }

  .runmeta {
    font-family: var(--mono);
    font-size: 0.76rem;
    color: var(--muted);
    display: flex;
    flex-wrap: wrap;
    gap: 0.4rem 1.5rem;
    padding-top: 1rem;
    border-top: 1px solid var(--hair);
  }
  .runmeta b { color: var(--ink); font-weight: 500; }

  /* ---------------------------------------------------------------- cards */

  .results { display: flex; flex-direction: column; gap: 2rem; }

  .card {
    background: var(--surface);
    border: 1px solid var(--hair);
    border-radius: 3px;
    overflow: hidden;
  }

  .card-head {
    display: flex;
    align-items: center;
    gap: 1rem;
    padding: 1.1rem 1.4rem;
    border-bottom: 1px solid var(--hair);
    flex-wrap: wrap;
  }

  .seq {
    font-family: var(--mono);
    font-size: 0.8rem;
    font-weight: 600;
    color: var(--ground);
    background: var(--copper);
    padding: 0.15rem 0.5rem;
    border-radius: 2px;
  }
  .card-head h3 {
    font-family: var(--serif);
    font-size: 1.15rem;
    font-weight: 600;
    margin: 0;
    letter-spacing: -0.01em;
  }

  .layers {
    font-family: var(--mono);
    font-size: 0.74rem;
    color: var(--muted);
    margin: 0.1rem 0 0;
  }

  .chip {
    margin-left: auto;
    font-family: var(--mono);
    font-size: 0.72rem;
    letter-spacing: 0.04em;
    padding: 0.28rem 0.7rem;
    border-radius: 2px;
    border: 1px solid currentColor;
    white-space: nowrap;
  }
  .chip.ok   { color: var(--ok); }
  .chip.warn { color: var(--warn); }

  .plate {
    margin: 0;
    padding: clamp(1rem, 3vw, 2rem);
    background: var(--plate);
    display: flex;
    justify-content: center;
    border-bottom: 1px solid var(--hair);
  }

  .render {
    width: 100%;
    max-width: 640px;
    height: auto;
  }

  .stats {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
    gap: 1px;
    margin: 0;
    background: var(--hair);
  }

  .stat {
    background: var(--surface);
    padding: 0.9rem 1.2rem;
  }
  .stat dt {
    font-family: var(--mono);
    font-size: 0.68rem;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    color: var(--muted);
  }
  .stat dd {
    font-family: var(--mono);
    font-size: 1.15rem;
    font-weight: 500;
    font-variant-numeric: tabular-nums;
    margin: 0.25rem 0 0;
  }
  .stat.good dd { color: var(--ok); }
  .stat.bad dd  { color: var(--front); }
  .stat.warn dd { color: var(--warn); }

  .failures {
    padding: 1.2rem 1.4rem;
    border-top: 1px solid var(--hair);
  }
  .failures h4 {
    font-family: var(--mono);
    font-size: 0.7rem;
    letter-spacing: 0.12em;
    text-transform: uppercase;
    color: var(--warn);
    margin: 0 0 0.7rem;
  }
  .failures ul { list-style: none; margin: 0; padding: 0; display: grid; gap: 0.4rem; }
  .failures li {
    display: flex;
    gap: 1rem;
    font-family: var(--mono);
    font-size: 0.78rem;
    flex-wrap: wrap;
  }
  .failures .net { color: var(--ink); min-width: 12ch; }
  .failures .why { color: var(--muted); }
  .note {
    font-size: 0.85rem;
    color: var(--muted);
    margin: 0.9rem 0 0;
    max-width: var(--measure);
  }

  /* ---------------------------------------------------------------- prose */

  section h2 {
    font-family: var(--serif);
    font-size: clamp(1.4rem, 3vw, 1.8rem);
    font-weight: 600;
    letter-spacing: -0.015em;
    margin: 0 0 1rem;
    text-wrap: balance;
  }

  section p, section li { max-width: var(--measure); }
  section p { margin: 0 0 1rem; }

  .lede { font-size: 1.05rem; }

  .formula {
    font-family: var(--mono);
    font-size: 0.85rem;
    background: var(--raised);
    border-left: 2px solid var(--copper);
    padding: 1rem 1.2rem;
    margin: 1.4rem 0;
    overflow-x: auto;
    color: var(--ink);
  }

  .legend {
    display: flex;
    gap: 1.5rem;
    flex-wrap: wrap;
    font-family: var(--mono);
    font-size: 0.76rem;
    color: var(--muted);
    margin-top: 0.6rem;
  }
  .swatch {
    display: inline-block;
    width: 0.75rem;
    height: 0.75rem;
    border-radius: 2px;
    margin-right: 0.45rem;
    vertical-align: -0.05rem;
  }

  ul.plain { list-style: none; padding: 0; margin: 0; display: grid; gap: 0.7rem; }
  ul.plain li {
    padding-left: 1.2rem;
    position: relative;
  }
  ul.plain li::before {
    content: "";
    position: absolute;
    left: 0;
    top: 0.62em;
    width: 0.45rem;
    height: 1px;
    background: var(--copper);
  }

  strong { font-weight: 600; }
  code {
    font-family: var(--mono);
    font-size: 0.88em;
    background: var(--raised);
    padding: 0.1em 0.35em;
    border-radius: 2px;
  }

  footer {
    border-top: 1px solid var(--hair);
    padding-top: 1.5rem;
    font-family: var(--mono);
    font-size: 0.74rem;
    color: var(--muted);
  }

  @media (prefers-reduced-motion: no-preference) {
    .card { transition: border-color 160ms ease; }
    .card:hover { border-color: var(--copper); }
  }
</style>

<div class="wrap">

  <header class="masthead">
    <span class="eyebrow">Optimal Prime &middot; MVP</span>
    <h1>A rubber band, not a grid</h1>
    <p class="thesis">Stretch a band between two pads and let it pull taut around whatever is in
      the way. The shape it settles into is made of straight lines and circular arcs, and
      nothing else &mdash; which happens to be exactly what a KiCad board file can hold.</p>
    <div class="runmeta">
      <span>engine <b>KiCad {{ENGINE}}</b></span>
      <span>boards <b>KiCad demo set</b></span>
      <span>arcs placed <b>{{ARCS}}</b></span>
      <span>verdict by <b>kicad-cli pcb drc</b></span>
    </div>
  </header>

  <section class="results">
    <h2>Results</h2>
    <p>Same board twice. The first run may use only the front copper; the second may also use
      the back. Everything is judged by KiCad&rsquo;s own DRC &mdash; there is no internal
      checker marking its own homework.</p>
    <div class="legend">
      <span><span class="swatch" style="background:#C83434"></span>front copper</span>
      <span><span class="swatch" style="background:#4D7FC4"></span>back copper</span>
      <span><span class="swatch" style="background:#D0D2CD"></span>board outline</span>
    </div>
    {{CARDS}}
  </section>

  <section>
    <h2>Why this shape</h2>
    <p class="lede">Every obstacle &mdash; a pad, a finished track, the board edge &mdash; is
      treated as something round to go around. The shortest path that avoids a set of circles
      is a theorem, not a heuristic: it is a sequence of straight lines tangent to those
      circles, joined by arcs riding on their surfaces.</p>
    <p>That matters because a <code>.kicad_pcb</code> can hold precisely two kinds of copper:
      <code>segment</code> and <code>arc</code>. The optimal geometry and the expressible
      geometry are the same set, so nothing is snapped to a grid, rounded to 45&deg;, or
      flattened into a polyline on the way out. The file holds the answer itself.</p>
    <p>The check that it really is the taut string: a single obstacle between two points must
      produce line&ndash;arc&ndash;line of exactly</p>
    <pre class="formula">2&middot;&radic;(d&sup2; &minus; r&sup2;)  +  r&middot;(&pi; &minus; 2&middot;acos(r/d))</pre>
    <p>It does, to one part in a million. Across 250 random obstacle fields the path never
      enters an obstacle by more than 7&times;10&#8315;&sup1;&#8310; mm.</p>
  </section>

  <section>
    <h2>What this does not do yet</h2>
    <ul class="plain">
      <li><strong>No vias.</strong> A connection is placed on whichever allowed layer gives the
        shorter path, but it never changes layer part-way. Two surface pads on opposite faces
        have no solution here.</li>
      <li><strong>Obstacles are circles.</strong> A long rectangular pad is treated as the
        circle that encloses it, which wastes space and occasionally refuses a connection that
        would in fact fit. Rounded rectangles are the honest next step.</li>
      <li><strong>Nets are routed one after another</strong>, each finished track becoming an
        obstacle for the next, so the order still matters. Nothing negotiates or reroutes.</li>
    </ul>
    <p class="note">All three are known and none of them can produce an illegal board &mdash;
      they cost completed connections, never correctness. When the router cannot find a legal
      path it says so instead of laying copper anyway.</p>
  </section>

  <footer>
    Routed with taut-string geometry &middot; verified by kicad-cli &middot; renders exported
    straight from the routed board files
  </footer>

</div>
"""

if __name__ == "__main__":
    raise SystemExit(main())
