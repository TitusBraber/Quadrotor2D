set(gcf, 'PaperUnits', 'centimeters', 'PaperPosition', [0 0  myset.norm.w  myset.norm.h], 'PaperSize', [ myset.norm.w  myset.norm.h] );
print('-painters',gcf,[ myset.path modelName testName subName 'norm.' myset.ext], myset.format);

set(gcf, 'PaperUnits', 'centimeters', 'PaperPosition', [0 0  myset.wide.w  myset.medium.h], 'PaperSize', [ myset.wide.w  myset.medium.h] );
print('-painters',gcf,[ myset.path modelName testName subName 'wide_medium.' myset.ext], myset.format);

set(gcf, 'PaperUnits', 'centimeters', 'PaperPosition', [0 0  myset.wide.w  myset.large.h], 'PaperSize', [ myset.wide.w  myset.large.h] );
print('-painters',gcf,[ myset.path modelName testName subName 'wide_large.' myset.ext], myset.format);

% set(gcf, 'PaperUnits', 'centimeters', 'PaperPosition', [0 0  myset.page.w  myset.page.h], 'PaperSize', [ myset.page.w  myset.page.h] );
% print(gcf,[ myset.path testName subName 'page.' myset.ext], myset.format);

% set(gcf, 'PaperUnits', 'centimeters', 'PaperPosition', [0 0  jfrset.norm.w  jfrset.norm.h], 'PaperSize', [ jfrset.norm.w  jfrset.norm.h] );
% print(gcf,[ jfrset.path 'ff_profile/eps.' jfrset.ext], jfrset.format);

% print('-painters',)

close all;