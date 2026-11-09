//! Oracle sweep of Jinja2 2.11.3's builtin filters and tests.
//!
//! Each case renders a template in the reference Jinja2 2.11.3 (Klipper's
//! `jinja2.Environment('{%', '%}', '{', '}')`, markupsafe 1.1.1) and in the
//! port against the same context, and compares the output or the raised
//! exception (class and `str()`). Every entry of `jinja2/filters.py`
//! `FILTERS` and `jinja2/tests.py` `TESTS` appears with representative
//! positional and keyword arguments, the argument-binding errors CPython
//! raises, the lazy generators (`map`, `select`, `unique`, `batch`, ...),
//! the `Markup` values `escape`/`safe`/`tojson` return and how operators
//! treat them, and the `%` formatting the `format` filter shares with the
//! `%` operator.
//!
//! A generator's or iterator's `repr()` carries its address in CPython;
//! the oracle side rewrites ` at 0x<hex>` to ` at 0x0`, the port's text.
//! `random` is only exercised where the outcome is deterministic.
//!
//! Integration owner runs (in the oracle image):
//!
//! ```sh
//! cargo test -p extras_macros_templates --test template_filters
//! ```

use extras_macros_templates::gcode_macro::{StatusSource, Template, Value};
use indexmap::IndexMap;
use pycompat::int::PyInt;
use pycompat::PyValue;
use recording::Value as RValue;
use std::path::PathBuf;

const CASES: &[&str] = &[
    // abs
    "{ -3|abs }{ -2.5|abs }{ true|abs }",
    "{ 'x'|abs }",
    "{ missing|abs }",
    "{ 1|abs(2) }",
    "{ 1|abs(x=1) }",
    // attr
    "{ d|attr('a') }|{ s|attr('upper')() }|{ n|attr('real') }",
    "{ missing|attr('x') }",
    "{ missing|attr('__x__') }",
    "{ (users|groupby('age'))[0]|attr('grouper') }",
    "{ l|attr() }",
    // batch
    "{ l|batch(2)|list }",
    "{ range(7)|batch(3, 'x')|list }",
    "{ l|batch(2) }",
    "{ 1|batch(2)|list }",
    "{ l|batch(0)|list }",
    "{ l|batch(2.0, 0)|list }",
    "{ l|batch }",
    // capitalize
    "{ 'hELLO wORLD'|capitalize }|{ n|capitalize }|{ ('<b>x'|e)|capitalize }|{ 'ß x'|capitalize }",
    // center
    "[{ 'abc'|center(9) }][{ 'ab'|center(7) }][{ 'abc'|center }]",
    "{ 'abc'|center(2.5) }",
    "{ 'abc'|center('x') }",
    // count / length
    "{ l|count }{ s|length }{ d|length }{ missing|length }",
    "{ n|length }",
    "{ printer|length }",
    "{ (l|map('abs'))|length }",
    // default
    "{ missing|default('x') }|{ none|default('x') }|{ ''|default('x', true) }|{ 0|d(5, boolean=true) }|{ missing|d }",
    "{ n|default(1, 2, 3) }",
    "{ n|default(foo=1) }",
    // dictsort
    "{ d|dictsort }|{ d|dictsort(true) }|{ d|dictsort(false, 'value') }|{ d|dictsort(reverse=true) }",
    "{ d|dictsort(by='x') }",
    "{ l|dictsort }",
    "{ missing|dictsort }",
    "{ {1: 'a', 'b': 2}|dictsort }",
    "{ d|dictsort(reverse='x') }",
    // escape
    "{ '<a href=\"x\">&\\'</a>'|e }|{ 5|escape }|{ none|e }|{ missing|e }|{ ('<'|e)|e }",
    "{ 1|e(2) }",
    // filesizeformat
    "{ 1|filesizeformat }|{ 300|filesizeformat }|{ 3000|filesizeformat }|{ 3000000|filesizeformat(true) }",
    "{ 1e30|filesizeformat }|{ '12345'|filesizeformat }|{ -5.5|filesizeformat }|{ 1e24|filesizeformat }|{ 1e27|filesizeformat }",
    "{ 1023|filesizeformat(binary=true) }|{ 1024|filesizeformat(binary=true) }",
    "{ 'x'|filesizeformat }",
    "{ none|filesizeformat }",
    // first
    "{ l|first }|{ s|first }|{ d|first }|{ []|first }|{ missing|first }|{ (l|map('abs'))|first }",
    "{ n|first }",
    // float
    "{ '1.5'|float }|{ 'x'|float }|{ 'x'|float(1) }|{ none|float(default=2) }|{ n|float }|{ ' 1e3 '|float }|{ 'nan'|float }",
    "{ missing|float }",
    "{ (10 ** 400)|float }",
    // forceescape
    "{ '<x>'|forceescape }|{ ('<x>'|e)|forceescape }|{ missing|forceescape }|{ 3|forceescape }",
    // format
    "{ '%s-%d'|format('a', 3) }|{ '%(a)s %(b)r'|format(a=1, b='x') }|{ '%5.1f|%-4d|%x'|format(3.14159, 7, 255) }",
    "{ '%s'|format([1, 2]) }|{ '%r %a'|format('é', 'é') }|{ '%c'|format(65) }|{ '%%|%s'|format(1) }",
    "{ '%*d|%.*f'|format(5, 3, 2, 3.14159) }|{ '%s'|format(missing) }|{ 5|format }",
    "{ '%s %s'|format(1) }",
    "{ '%s'|format(1, 2) }",
    "{ '%s'|format(1, x=2) }",
    "{ '%y'|format(1) }",
    "{ '%'|format() }",
    "{ '%d'|format('x') }",
    "{ ('<b>%s</b>'|safe)|format('<i>') }",
    "{ '%(a)s'|format(1) }",
    "{ '%(z)s'|format(a=1) }",
    "{ '%d'|format(missing) }",
    // groupby
    "{ users|groupby('age') }",
    "{ users|groupby('city') }",
    "{% for g in users|groupby('age') %}{ g.grouper }:{ g.list|map(attribute='name')|join(',') };{ g[0] }{% endfor %}",
    "{ users|groupby('missing') }",
    "{ l|groupby(0) }",
    "{ [[1, 2], [1, 3], [2, 4]]|groupby(0) }",
    "{ users|groupby }",
    // indent
    "{ 'a\\nb\\n\\nc'|indent }",
    "{ 'a\\nb\\n\\nc'|indent(2, true) }",
    "{ 'a\\nb\\n\\nc'|indent(2, blank=true) }",
    "{ 'a\\nb'|indent(first=true) }|{ 'a\\nb'|indent(indentfirst=true) }|{ ''|indent(2, true) }",
    "{ 5|indent }",
    "{ missing|indent }",
    "{ 'a\\nb'|indent(1.5) }",
    "{ ('<a\\nb'|e)|indent(1, true) }",
    // int
    "{ '42'|int }|{ '42.9'|int }|{ 'x'|int }|{ 'x'|int(7) }|{ '0x1A'|int(0, 16) }|{ '0b101'|int(base=0) }",
    "{ 4.9|int }|{ -4.9|int }|{ true|int }|{ none|int }|{ 'nan'|int }|{ ' 12 '|int }|{ '1_000'|int }",
    "{ [1]|int(default='d') }|{ '12'|int(base=1.5) }|{ '1e3'|int }|{ 'ff'|int(base=16) }|{ '12'|int(0, 40) }",
    "{ 'inf'|int }",
    "{ missing|int }",
    // join
    "{ l|join }|{ l|join('|') }|{ users|join(', ', attribute='name') }|{ [1, none, 'x']|join('-') }|{ s|join('.') }",
    "{ users|join(d='/', attribute='age') }|{ [missing, 1]|join(',') }",
    "{ n|join }",
    // last
    "{ l|last }|{ s|last }|{ d|last }|{ []|last }|{ missing|last }|{ ('ab'|e)|last }",
    "{ n|last }",
    "{ (l|map('abs'))|last }",
    // list
    "{ s|list }|{ d|list }|{ missing|list }|{ (1, 2)|list }|{ (l|reverse)|list }",
    "{ n|list }",
    // lower / upper
    "{ s|lower }|{ s|upper }|{ n|upper }|{ missing|upper }|{ ('<A>'|e)|lower }|{ 'ß'|upper }",
    "{ s|lower(1) }",
    // map
    "{ users|map(attribute='name')|list }|{ users|map(attribute='nick', default='?')|list }",
    "{ l|map('string')|list }|{ ['1', '2']|map('int')|sum }|{ [1.25, 2.5]|map('round', 1)|list }|{ words|map('upper')|join }",
    "{ users|map('attr', 'name')|list }|{ users|map(attribute='age')|max }|{ missing|map('int')|list }",
    "{ l|map }",
    "{ l|map|list }",
    "{ l|map('nosuch')|list }",
    "{ []|map('nosuch')|list }",
    "{ l|map(missing)|list }",
    "{ users|map(attribute='name', x=1)|list }",
    "{ l|map(5)|list }",
    "{ l|map([1])|list }",
    "{ n|map('int')|list }",
    // min / max
    "{ l|min }|{ l|max }|{ words|min }|{ words|max(true) }|{ users|max(attribute='age') }|{ users|min(attribute='name') }",
    "{ []|min }|{ s|max }|{ [1, 'nan'|float, 3]|max }",
    "{ [1, 'a']|max }",
    "{ [1, 'a']|min }",
    "{ n|min }",
    // pprint
    "{ d|pprint }|{ l|pprint }|{ s|pprint }|{ users|pprint }|{ none|pprint }|{ ('<'|e)|pprint }",
    "{ {'k': ['x' * 30, 'y' * 30, 'z' * 30]}|pprint }",
    "{ ('word ' * 30)|pprint }",
    "{ [('a' * 40, 'b' * 40)]|pprint }",
    "{ {2: 'b', 1: 'a', 'x': 3}|pprint }",
    "{ users|groupby('age')|pprint }",
    "{ d|pprint(true) }",
    "{ (('line one ' * 5) ~ '\\n' ~ ('line two ' * 5))|pprint }",
    // random
    "{ [7]|random }|{ []|random }|{ 'z'|random }|{ missing|random }",
    "{ n|random }",
    "{ {'a': 1}|random }",
    // select / reject / selectattr / rejectattr
    "{ range(10)|select('odd')|list }|{ range(10)|reject('odd')|list }|{ range(10)|select('divisibleby', 3)|list }",
    "{ [0, 1, '', 'x', none]|select|list }|{ [0, 1, '', 'x', none]|reject|list }",
    "{ users|selectattr('age', 'gt', 26)|map(attribute='name')|list }|{ users|rejectattr('city', 'equalto', 'Paris')|map(attribute='name')|list }",
    "{ users|selectattr('name')|list|length }|{ l|select('in', [1, 2])|list }|{ l|select('lessthan', 3)|list }",
    "{ l|select('==', 1)|list }|{ l|select('>', 1)|list }|{ l|reject('sameas', 1)|list }",
    "{ l|select('nosuch')|list }",
    "{ []|select('nosuch')|list }",
    "{ l|selectattr|list }",
    "{ l|select('odd', 1)|list }",
    "{ l|select }",
    "{ missing|select('odd')|list }",
    // replace
    "{ s|replace('l', 'L') }|{ s|replace('l', 'L', 2) }|{ s|replace('', '-') }|{ s|replace('', '-', 3) }|{ n|replace(5, 6) }|{ s|replace('l', 'L', count=1) }",
    "{ s|replace('l') }",
    "{ s|replace('l', 'L', 1.5) }",
    "{ ('<x>'|e)|replace('&lt;', '<') }",
    // reverse
    "{ s|reverse }|{ l|reverse|list }|{ l|reverse }|{ (1, 2)|reverse }|{ d|reverse }|{ (l|map('abs'))|reverse }",
    "{ ('<ab'|e)|reverse }|{ missing|reverse }|{ missing|reverse|list }|{ printer|reverse }",
    "{ n|reverse }",
    // round
    "{ 42.55|round }|{ 42.55|round(1, 'floor') }|{ 42.51|round(1, 'ceil') }|{ 2.5|round }|{ 3.5|round }|{ 2.675|round(2) }",
    "{ 15|round(-1) }|{ 25|round(-1) }|{ 1234|round(-2, 'ceil') }|{ 5|round(1) }|{ true|round }|{ 2.5|round(none) }",
    "{ 1.23456|round(-1, 'floor') }|{ 7|round(method='floor') }|{ 1.5|round(2, method='ceil') }",
    "{ 1.5|round(1.5) }",
    "{ 'a'|round }",
    "{ missing|round }",
    "{ 2.5|round(method='x') }",
    "{ 2.5|round(method=[1]) }",
    "{ 'a'|round(0, 'ceil') }",
    "{ (s|replace(s, 'nan')|float)|round(0, 'floor') }",
    "{ (s|replace(s, 'inf')|float)|round(none) }",
    // safe and Markup operators
    "{ '<b>'|safe }|{ ('<b>'|safe) + '<i>' }|{ '<i>' + ('<b>'|safe) }|{ ('<b>'|safe) ~ '<i>' }|{ ['<'|safe] }|{ ('<'|safe) * 3 }",
    "{ missing|safe }|{ 5|safe }|{ 2 * ('<'|e) }|{ ('<'|e) == '&lt;' }|{ ('<'|e) in 'a&lt;b' }|{ 'lt' in ('<'|e) }",
    "{ ('<b>'|e)[1] }|{ ('<b>'|e).upper() }|{ ('a<b'|safe).split('<') }|{ (', '|safe).join(['<', '>']) }",
    "{ ('a &amp; b'|safe).unescape() }|{ ('<b>x</b>'|safe).striptags() }|{ ('%s'|safe) % ('<',) }|{ ('%(a)s'|safe) % {'a': '<'} }|{ ('%d'|safe) % '5' }",
    "{ ('<'|e) * 'x' }",
    "{ 'x' * ('<'|e) }",
    "{ ('<'|e) * 2.5 }",
    "{ ('<'|e) * ('<'|e) }",
    "{ (l|map('abs')|list) * 2 }|{ 2 * (['<'|e]) }",
    "{ ['<'|e] * 'x' }",
    "{ printer|list }|{ printer|first }|{ printer|join(',') }|{ printer|select|list }",
    "{ printer|tojson }",
    "{ ('<b>'|e).title() }|{ ('  <b> '|safe).strip() }|{ ('a<b'|safe).replace('<', '>') }|{ ('ab'|safe).startswith(('x', 'a')) }",
    "{ ('<'|e) + 5 }",
    "{ 5 + ('<'|e) }",
    // slice
    "{ range(10)|slice(3)|list }|{ range(10)|slice(3, 'x')|list }|{ l|slice(4)|list }",
    "{ l|slice(0)|list }",
    "{ l|slice(2.0)|list }",
    "{ l|slice }",
    // sort
    "{ l|sort }|{ words|sort }|{ words|sort(case_sensitive=true) }|{ words|sort(reverse=true) }",
    "{ users|sort(attribute='age')|map(attribute='name')|list }|{ users|sort(attribute='age,name')|map(attribute='name')|list }",
    "{ users|sort(attribute='city', reverse=true)|map(attribute='name')|list }|{ [3, 1.5, true]|sort }|{ s|sort|join }|{ d|sort }",
    "{ ['nan'|float, 1, 0]|sort }|{ [2, 'nan'|float, 1, 0, -1]|sort(reverse=true) }",
    "{ [1, 'a']|sort }",
    "{ [2, 1]|sort(reverse='x') }",
    "{ n|sort }",
    "{ [[2, 'a'], [1, 'b'], [2, 1]]|sort }",
    // string
    "{ n|string }|{ none|string }|{ missing|string }|{ ('<'|e)|string }|{ l|string }",
    // striptags
    "{ html|striptags }",
    "{ '<a\\nb>x</a>  y'|striptags }|{ '&lt;tag&gt; &unknown; &#x110000; &#1234; &#x41;'|striptags }|{ 5|striptags }",
    "{ ('<b>x</b>'|e)|striptags }|{ '<!--\\n-->z'|striptags }|{ 'a<b'|striptags }",
    // sum
    "{ l|sum }|{ [1.5, 2]|sum(start=1) }|{ users|sum(attribute='age') }|{ [[1], [2]]|sum(start=[]) }|{ []|sum }",
    "{ ['a']|sum }",
    "{ l|sum(start='x') }",
    "{ n|sum }",
    "{ [missing]|sum }",
    // title
    "{ 'hello-world (foo)bar [baz]{qux}<a>b'|title }|{ \"it's me o'neil\"|title }|{ ('<b>x'|e)|title }|{ n|title }",
    // trim
    "[{ '  x  '|trim }][{ '--x--'|trim('-') }][{ ('<x<'|e)|trim('<') }][{ n|trim }]",
    "{ 'xx'|trim(5) }",
    // truncate
    "{ 'foo bar baz qux'|truncate(9) }|{ 'foo bar baz qux'|truncate(9, true) }|{ 'foo bar baz qux'|truncate(11) }",
    "{ 'foo bar baz qux'|truncate(11, false, '...', 0) }|{ 'foo bar baz qux'|truncate(9, end='!') }|{ l|truncate(3) }",
    "{ ('<a> <b> <c> <d>'|e)|truncate(9, leeway=0) }|{ missing|truncate }",
    "{ s|truncate(2) }",
    "{ s|truncate(5, leeway=-1) }",
    "{ range(300)|list|truncate(3) }",
    "{ 5|truncate }",
    "{ 'x y z'|truncate(4.0, leeway=0) }",
    // unique
    "{ ['a', 'A', 'b']|unique|list }|{ ['a', 'A', 'b']|unique(true)|list }|{ users|unique(attribute='age')|map(attribute='name')|list }",
    "{ [1, 1.0, true, 2]|unique|list }|{ l|unique }",
    "{ [[1], [1]]|unique|list }",
    "{ n|unique|list }",
    // urlencode
    "{ 'a b/c&d=é'|urlencode }|{ d|urlencode }|{ [('a', 1), ('b c', 'd&e')]|urlencode }|{ 5|urlencode }|{ none|urlencode }|{ missing|urlencode }|{ ['ab']|urlencode }",
    "{ [1]|urlencode }",
    "{ ['abc']|urlencode }",
    // urlize
    "{ 'see www.example.com, or http://x.org/a. mail foo@bar.com (https://y.com/q?a=1&b=2)'|urlize }",
    "{ 'go to example.org now'|urlize }|{ 'http://averyveryverylongdomain.example.com/path'|urlize(10) }",
    "{ 'http://x.org'|urlize(nofollow=true, target='_blank') }|{ 'http://x.org'|urlize(rel='me nofollow') }",
    "{ '<script>http://x.org</script>'|urlize }|{ 'a\\nhttp://x.org\\n'|urlize }|{ 'foo@bar'|urlize }|{ ''|urlize }",
    "{ 'http://x.org'|urlize(-3) }|{ ' lead http://a.com trail '|urlize }",
    "{ 'http://x.org'|urlize(5.5) }",
    // wordcount
    "{ 'hello world, foo_bar 42!'|wordcount }|{ n|wordcount }|{ ''|wordcount }",
    // wordwrap
    "{ text|wordwrap(10) }",
    "{ text|wordwrap(10, break_on_hyphens=false) }",
    "{ text|wordwrap(5, false) }",
    "{ 'a\\n\\nb c'|wordwrap(3) }|{ 'abcdefghij'|wordwrap(3) }|{ 'x -- y --z'|wordwrap(3) }",
    "{ text|wordwrap(10, wrapstring='<br>') }",
    "{ text|wordwrap(10, wrapstring=('<br>'|safe)) }",
    "{ text|wordwrap(12.5) }",
    "{ text|wordwrap(0) }",
    "{ 'abcdefghij'|wordwrap(3.5) }",
    "{ 5|wordwrap }",
    // xmlattr
    "{ {'class': 'a<', 'id': 3, 'x': none}|xmlattr }|{ {'a': 1}|xmlattr(false) }|{ {}|xmlattr }",
    "{ l|xmlattr }",
    "{ missing|xmlattr }",
    // tojson
    "{ d|tojson }|{ users|tojson }|{ l|tojson(indent=0) }|{ []|tojson(2) }|{ {}|tojson(2) }",
    "{ {'b': '<a>&\\'', 'a': [1, 2.5, none, true]}|tojson(2) }",
    "{ s|tojson('--') }|{ 'é'|tojson }|{ ('nan'|float)|tojson }|{ ('<'|e)|tojson }|{ (1, 2)|tojson }",
    "{ missing|tojson }",
    "{ {1: 'a', 'b': 2}|tojson }",
    "{ l|tojson(1.5) }",
    // tests: numbers
    "{ 3 is odd }{ 4 is even }{ 3.0 is odd }{ 9 is divisibleby 3 }{ 9 is divisibleby(4) }{ '%d' is even }",
    "{ 'abc' is odd }",
    "{ none is odd }",
    "{ missing is odd }",
    "{ 9 is divisibleby 0 }",
    "{ 9.0 is divisibleby 0 }",
    // tests: definedness and types
    "{ missing is defined }{ n is defined }{ missing is undefined }{ none is none }{ 0 is none }",
    "{ true is boolean }{ 1 is boolean }{ false is false }{ 0 is false }{ true is true }",
    "{ 1 is integer }{ true is integer }{ 1.0 is float }{ 1 is number }{ true is number }{ '1' is number }",
    "{ 'abc' is lower }{ 'aBc' is lower }{ 'ABC' is upper }{ '123' is upper }{ missing is lower }{ l is lower }",
    "{ s is string }{ ('x'|e) is string }{ d is mapping }{ printer is mapping }{ l is sequence }{ d is sequence }",
    "{ missing is sequence }{ n is sequence }{ printer is sequence }{ (l|map('abs')) is sequence }",
    "{ l is iterable }{ n is iterable }{ missing is iterable }{ printer is iterable }{ (l|map('abs')) is iterable }",
    "{ s.upper is callable }{ s is callable }{ missing is callable }{ printer is callable }{ range is callable }",
    "{ none is sameas none }{ 1 is sameas 1 }{ true is sameas true }{ 1 is sameas true }{ 1000 is sameas 1000 }{ 'a' is sameas 'a' }{ printer is sameas printer }",
    "{ ('x'|e) is escaped }{ 'x' is escaped }{ missing is escaped }",
    "{ 1 is in l }{ 'a' is in 'cat' }{ 'z' is in d }{ 1 is in missing }",
    "{ 1 is in n }",
    "{ 1 is eq 1 }{ 1 is equalto 1.0 }{ 'a' is ne 'b' }{ 2 is gt 1 }{ 2 is greaterthan 3 }{ 2 is ge 2 }{ 1 is lt 2 }{ 1 is lessthan 0 }{ 1 is le 1 }",
    "{ 3 is not odd }{ missing is not defined }",
    "{ 1 is gt 'a' }",
    "{ 1 is eq }",
    "{ 1 is eq(1, 2) }",
    "{ 1 is eq(x=1) }",
    "{ 1 is callable(2) }",
    "{ 1 is odd(2) }",
    "{ 1 is divisibleby }",
    "{ 1 is defined(x=1) }",
    "{ missing is gt 1 }",
    // the `%` operator
    "{ '%s %r' % ('a', [1]) }|{ '%s' % [1, 2] }|{ 'abc' % [1] }|{ '%(a)s' % {'a': 1} }|{ '%s' % missing }|{ '%5.2f' % 3.14159 }",
    "{ 'abc' % 5 }",
    "{ '%5s|%-5s|%x|%.2e|%r|%c|%+d|%05.1f' % ('a', 'b', 255, 12345.678, 'x', 65, 3, 2.25) }",
    "{ '%s %s' % (1,) }",
    "{ '%(a)s %s' % {'a': 1} }",
    "{ '%c' % 'ab' }",
    "{ '%d' % [1] }",
    "{ '%(a)s' % [1] }",
    "{ '%s' % (missing, ) }|{ '%r' % (missing, ) }",
];

const PY_SETUP: &str = r#"
import sys, re, contextlib
sys.path.insert(0, '__KLIPPY__')
import jinja2
import extras.gcode_macro as gcode_macro

class _Reactor:
    def monotonic(self):
        return 1.0
    def assert_no_pause(self):
        return contextlib.nullcontext()

class _Obj:
    def __init__(self, status):
        self.status = status
    def get_status(self, eventtime):
        return dict(self.status)

class _Printer:
    def __init__(self):
        self.objects = {'toolhead': _Obj({'position': [1.0, 2.0]})}
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)
    def lookup_objects(self):
        return list(self.objects.items())
    def get_reactor(self):
        return _Reactor()

_env = jinja2.Environment('{%', '%}', '{', '}')
_printer = gcode_macro.GetStatusWrapper(_Printer())

def render_case(src):
    ctx = dict(
        l=[3, 1, 2], s='Hello World', n=5, f=2.5, none=None,
        d={'b': 1, 'a': 2, 'C': 3},
        users=[{'name': 'bob', 'age': 30, 'city': 'Paris'},
               {'name': 'Alice', 'age': 25, 'city': 'Berlin'},
               {'name': 'carl', 'age': 30, 'city': 'paris'}],
        words=['b', 'A', 'a', 'B'],
        text='Look, goof-ball -- use the -b option! and some-more hyphen-a-ted words',
        html='<p>Hello &amp; <b>world</b> &nbsp;&#65;&#x42;&bogus; <!-- c -->x</p>',
        printer=_printer)
    try:
        out = _env.from_string(src).render(ctx)
    except Exception as e:
        out = '%s: %s' % (type(e).__name__, e)
    return re.sub(r' at 0x[0-9a-f]+', ' at 0x0', out)
"#;

/// The `_Printer` objects of the oracle side.
struct Statuses;

impl StatusSource for Statuses {
    fn object_names(&self) -> Vec<String> {
        vec!["toolhead".to_string()]
    }
    fn get_status(
        &self,
        name: &str,
        eventtime: &mut Option<f64>,
    ) -> Result<Option<PyValue>, extras_macros_templates::gcode_macro::TemplateError> {
        if name != "toolhead" {
            return Ok(None);
        }
        eventtime.get_or_insert(1.0);
        Ok(Some(dict(vec![("position", PyValue::List(vec![PyValue::Float(1.0), PyValue::Float(2.0)]))])))
    }
}

fn int(n: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(n))
}

fn text(s: &str) -> PyValue {
    PyValue::Str(s.to_string())
}

fn dict(pairs: Vec<(&str, PyValue)>) -> PyValue {
    PyValue::Dict(pairs.into_iter().map(|(k, v)| (text(k), v)).collect())
}

fn user(name: &str, age: i64, city: &str) -> PyValue {
    dict(vec![("name", text(name)), ("age", int(age)), ("city", text(city))])
}

/// The oracle's `render_case` context.
fn context() -> IndexMap<String, Value> {
    let mut ctx = IndexMap::new();
    let mut put = |k: &str, v: PyValue| {
        ctx.insert(k.to_string(), Value::py(v));
    };
    put("l", PyValue::List(vec![int(3), int(1), int(2)]));
    put("s", text("Hello World"));
    put("n", int(5));
    put("f", PyValue::Float(2.5));
    put("none", PyValue::None);
    put("d", dict(vec![("b", int(1)), ("a", int(2)), ("C", int(3))]));
    put(
        "users",
        PyValue::List(vec![user("bob", 30, "Paris"), user("Alice", 25, "Berlin"), user("carl", 30, "paris")]),
    );
    put("words", PyValue::List(vec![text("b"), text("A"), text("a"), text("B")]));
    put("text", text("Look, goof-ball -- use the -b option! and some-more hyphen-a-ted words"));
    put("html", text("<p>Hello &amp; <b>world</b> &nbsp;&#65;&#x42;&bogus; <!-- c -->x</p>"));
    ctx.insert("printer".to_string(), Value::printer_status(Box::new(Statuses), None));
    ctx
}

fn klippy_path() -> String {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../klippy")
        .to_string_lossy()
        .replace('\\', "/")
}

/// A Python string literal for `s`.
fn py_literal(s: &str) -> String {
    let mut out = String::from("'");
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\'' => out.push_str("\\'"),
            ' '..='~' => out.push(c),
            _ => out.push_str(&format!("\\U{:08x}", c as u32)),
        }
    }
    out.push('\'');
    out
}

#[test]
fn builtin_filters_and_tests_match_jinja() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    if let Err(e) = py.exec(&PY_SETUP.replace("__KLIPPY__", &klippy_path())) {
        panic!("oracle setup: {}: {}", e.class_name, e.message);
    }
    let mut failures = Vec::new();
    for src in CASES {
        let expected = match py.eval(&format!("render_case({})", py_literal(src))) {
            Ok(RValue::Str(s)) => s,
            other => panic!("oracle at {src:?}: {other:?}"),
        };
        let template = Template::parse(src).unwrap_or_else(|e| panic!("{src:?}: {e}"));
        let got = match template.render(context()) {
            Ok(out) => out,
            Err(e) => format!("{}: {}", e.kind(), e.message()),
        };
        if got != expected {
            failures.push(format!("{src:?}:\n  jinja {expected:?}\n  rust  {got:?}"));
        }
    }
    assert!(
        failures.is_empty(),
        "{} of {} cases differ:\n{}",
        failures.len(),
        CASES.len(),
        failures.join("\n")
    );
}
