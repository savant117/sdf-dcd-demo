// Theme toggle, the click to load demo and the BibTeX copy button
(function () {
    var root = document.documentElement;

    document.getElementById('theme-toggle').addEventListener('click', function () {
        var current = root.getAttribute('data-theme');
        var dark = current ? current === 'dark' : matchMedia('(prefers-color-scheme: dark)').matches;
        var next = dark ? 'light' : 'dark';
        root.setAttribute('data-theme', next);
        try { localStorage.setItem('theme', next); } catch (e) { }
    });

    // The demo is only loaded on request, since it downloads and compiles a few MB of WebAssembly
    var frame = document.getElementById('demo-frame');
    var chips = document.querySelectorAll('#scene-links [data-scene]');
    function launch(scene) {
        var src = 'demo/index.html' + (scene > 0 ? '?scene=' + scene : '');
        var iframe = frame.querySelector('iframe');
        if (!iframe) {
            iframe = document.createElement('iframe');
            iframe.title = 'Interactive SDF collision demo';
            iframe.allow = 'fullscreen';
            frame.textContent = '';
            frame.appendChild(iframe);
        }
        iframe.src = src;
        iframe.focus();
        document.getElementById('demo-fullscreen').href = src;
        chips.forEach(function (chip) { chip.classList.toggle('active', chip.dataset.scene == scene); });
    }
    document.getElementById('demo-launch').addEventListener('click', function () { launch(0); });
    chips.forEach(function (chip) {
        chip.addEventListener('click', function () { launch(parseInt(chip.dataset.scene)); });
    });

    var copy = document.getElementById('copy-bibtex');
    copy.addEventListener('click', function () {
        var text = document.getElementById('bibtex').textContent.replace(copy.textContent, '').trim();
        navigator.clipboard.writeText(text).then(function () {
            copy.textContent = 'Copied';
            setTimeout(function () { copy.textContent = 'Copy'; }, 1500);
        });
    });
})();
